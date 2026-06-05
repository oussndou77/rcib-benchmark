#!/bin/bash
# launch_carla.sh — Lance le serveur CARLA headless puis le smoke test.
# À exécuter DANS le pod RunPod APRÈS setup_runpod.sh.
#
# Leçons de la Phase 0 intégrées ici :
#   - CARLA met du temps à démarrer (~60-90s) : ne pas confondre "lent" et "planté"
#   - -RenderOffScreen fonctionne sur RTX 3090, mais si crash Vulkan → fallback -nullrhi
#   - le port 2000 ne doit être utilisé que par UN serveur (sinon "Address already in use")
#
# Usage :
#   source ~/rcib_env.sh        # d'abord charger l'environnement
#   bash launch_carla.sh        # lance serveur + attend + smoke test
#   bash launch_carla.sh --nullrhi   # force le mode sans rendu GPU

set -e

CARLA_DIR="${CARLA_DIR:-/home/carla}"
RPC_PORT="${RPC_PORT:-2000}"
WAIT_SECONDS="${WAIT_SECONDS:-90}"   # CARLA est lent à démarrer : 90s par défaut

# Mode de rendu : -RenderOffScreen par défaut, -nullrhi si demandé ou en fallback
RENDER_FLAG="-RenderOffScreen"
if [ "$1" == "--nullrhi" ]; then
    RENDER_FLAG="-nullrhi"
fi

echo "================================================================"
echo "  Lancement CARLA ($RENDER_FLAG) + smoke test"
echo "================================================================"

# ── 0. Nettoyer tout serveur CARLA résiduel (évite le conflit de port) ──
pkill -9 -f CarlaUE4 2>/dev/null || true
sleep 3

# ── 1. Lancer le serveur en arrière-plan ──
echo ""
echo "[1] Démarrage du serveur CARLA ($RENDER_FLAG, port $RPC_PORT)..."
cd "$CARLA_DIR"
./CarlaUE4.sh $RENDER_FLAG -nosound -carla-rpc-port=$RPC_PORT > ~/carla_server.log 2>&1 &
sleep 5
# Récupérer le PID du binaire réel (pas du script shell)
echo "    Serveur lancé. Attente du démarrage ($WAIT_SECONDS s)..."

# ── 2. Attendre, en vérifiant que le binaire tourne ──
for i in $(seq 1 $WAIT_SECONDS); do
    sleep 1
    if [ $((i % 15)) -eq 0 ]; then
        printf " %ss" "$i"
    fi
done
echo ""

if ! ps aux | grep CarlaUE4-Linux-Shipping | grep -v grep > /dev/null; then
    echo "✗ Le serveur CARLA ne tourne pas. Dernières lignes du log :"
    tail -15 ~/carla_server.log
    if [ "$RENDER_FLAG" != "-nullrhi" ]; then
        echo ""
        echo "→ Crash probable de rendu Vulkan. Réessaie avec : bash launch_carla.sh --nullrhi"
    fi
    exit 1
fi
echo "[2] ✓ Le serveur CARLA tourne."

# ── 3. Smoke test de connexion ──
echo ""
echo "[3] Smoke test (connexion client → serveur)..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/smoke_test.py" --port $RPC_PORT --timeout 30
exit $?
