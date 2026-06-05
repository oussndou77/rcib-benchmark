#!/bin/bash
# setup_runpod.sh — Prépare un pod RunPod frais (image carlasim/carla:0.9.15)
# pour faire tourner le client Python CARLA + nos scripts RCIB.
#
# Pourquoi ce script existe : l'image carlasim/carla:0.9.15 tourne sous l'utilisateur
# 'carla' SANS root, avec Python 3.6 (alors que le wheel carla est en 3.7), et il lui
# manque 3 libs système (libjpeg, libtiff, libjbig). On ne peut pas faire 'apt install'
# (pas de root), mais on PEUT télécharger les .deb et les extraire localement.
# Ce script automatise tout ce qu'on a découvert à la main lors de la Phase 0.
#
# Usage (dans un pod frais, connecté en SSH en tant que 'carla') :
#   bash setup_runpod.sh
#   source ~/rcib_env.sh        # charge les variables d'env (PYTHONPATH, LD_LIBRARY_PATH...)
#
# IMPORTANT : le container disk RunPod est effacé à chaque arrêt du pod.
# Il faut donc relancer ce script à chaque nouveau pod (jusqu'à ce qu'on ait
# une image Docker custom qui embarque tout — voir docker/Dockerfile.runpod).

set -e

echo "================================================================"
echo "  Setup RunPod pour CARLA 0.9.15 (sans root)"
echo "================================================================"

LIBDIR="$HOME/locallibs"
DEBDIR="$HOME/debs"
mkdir -p "$LIBDIR" "$DEBDIR"
cd "$DEBDIR"

# ── 1. Télécharger les .deb des libs système manquantes (sans root) ──
# apt-get download écrit dans le dossier courant : pas besoin de privilèges.
echo ""
echo "[1] Téléchargement des libs système (libjpeg, libtiff, libjbig)..."
for pkg in libjpeg-turbo8 libtiff5 libjbig0; do
    if ls ${pkg}_*.deb >/dev/null 2>&1; then
        echo "    $pkg : déjà téléchargé"
    else
        echo "    $pkg : téléchargement..."
        apt-get download "$pkg"
    fi
done

# ── 2. Extraire les .deb localement (sans root) ──
echo ""
echo "[2] Extraction des libs dans $LIBDIR..."
for deb in *.deb; do
    dpkg -x "$deb" "$LIBDIR"
done

# Vérifier que les 3 libs critiques sont là
echo ""
echo "[3] Vérification des libs extraites :"
ALL_OK=1
for lib in libjpeg.so.8 libtiff.so.5 libjbig.so.0; do
    if find "$LIBDIR" -name "$lib" | grep -q .; then
        echo "    ✓ $lib"
    else
        echo "    ✗ $lib MANQUANTE"
        ALL_OK=0
    fi
done

# ── 4. Localiser l'egg Python CARLA dans l'image ──
EGG=$(ls /home/carla/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg 2>/dev/null | head -n1)
if [ -z "$EGG" ]; then
    echo "    ✗ egg carla py3.7 introuvable dans l'image !"
    ALL_OK=0
else
    echo "    ✓ egg carla : $EGG"
fi

# ── 4b. Backport 'dataclasses' pour Python 3.6 ──
# L'image tourne en Python 3.6 ; le module 'dataclasses' (stdlib à partir de 3.7)
# est absent. Le code RCIB l'utilise partout. On récupère le backport officiel
# (un seul fichier pur Python, par l'auteur des dataclasses CPython). Sans pip.
PYLIB="$HOME/pylibs"
mkdir -p "$PYLIB"
PYVER=$(python3 -c "import sys; print('%d.%d' % sys.version_info[:2])")
if python3 -c "import dataclasses" 2>/dev/null; then
    echo "    ✓ dataclasses déjà présent (Python $PYVER)"
else
    echo "    dataclasses absent (Python $PYVER) : récupération du backport..."
    python3 -c "import urllib.request; urllib.request.urlretrieve('https://raw.githubusercontent.com/ericvsmith/dataclasses/master/dataclasses.py', '$PYLIB/dataclasses.py')" \
        && echo "    ✓ dataclasses backport installé dans $PYLIB" \
        || { echo "    ✗ échec du téléchargement de dataclasses"; ALL_OK=0; }
fi

# ── 5. Générer le fichier d'environnement à sourcer ──
ENVFILE="$HOME/rcib_env.sh"
cat > "$ENVFILE" << EOF
# Variables d'environnement RCIB (généré par setup_runpod.sh)
# À sourcer dans chaque shell : source ~/rcib_env.sh
export LD_LIBRARY_PATH=$LIBDIR/usr/lib/x86_64-linux-gnu:\$LD_LIBRARY_PATH
export PYTHONPATH=$EGG:$PYLIB:\$PYTHONPATH
export LC_ALL=C.UTF-8
export LANG=C.UTF-8
EOF
echo ""
echo "[4] Fichier d'environnement écrit : $ENVFILE"

# ── 6. Test d'import ──
echo ""
echo "[5] Test d'import de carla..."
# shellcheck disable=SC1090
source "$ENVFILE"
if python3 -c "import carla; carla.Client('localhost', 2000)" 2>/dev/null; then
    echo "    ✓✓✓ import carla OK — l'API Python est fonctionnelle."
else
    echo "    ✗ l'import a échoué — une lib supplémentaire manque peut-être."
    echo "      Lance : python3 -c 'import carla' pour voir le nom de la lib,"
    echo "      puis : cd ~/debs && apt-get download <paquet> && dpkg -x <deb> ~/locallibs"
    ALL_OK=0
fi

echo ""
echo "================================================================"
if [ "$ALL_OK" = "1" ]; then
    echo "  ✓ SETUP TERMINÉ. Pour utiliser carla dans un shell :"
    echo "      source ~/rcib_env.sh"
    echo ""
    echo "  Puis lance le serveur CARLA (laisse-lui ~90s pour démarrer) :"
    echo "      cd /home/carla"
    echo "      ./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000 &"
    echo ""
    echo "  NOTE : si -RenderOffScreen crashe (Vulkan), utilise -nullrhi à la place."
    echo "  Le serveur met du temps à démarrer : attends avant de connecter le client."
else
    echo "  ⚠ SETUP INCOMPLET — voir les messages ci-dessus."
fi
echo "================================================================"
