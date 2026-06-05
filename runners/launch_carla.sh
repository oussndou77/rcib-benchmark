#!/bin/bash
# launch_carla.sh — Phase 0 : lance le serveur CARLA headless puis le smoke test.
# À exécuter DANS le pod RunPod (image carlasim/carla:0.9.15).
#
# Usage :
#   bash launch_carla.sh           # lance serveur + attend + smoke test
#   bash launch_carla.sh --server-only   # lance juste le serveur (pour debug)

set -e

CARLA_DIR="${CARLA_DIR:-/home/carla}"
RPC_PORT="${RPC_PORT:-2000}"
WAIT_SECONDS="${WAIT_SECONDS:-45}"   # CARLA met ~30-60s à être prêt

echo "================================================================"
echo "  Phase 0 — Lancement CARLA headless + smoke test"
echo "================================================================"

# ── 1. Lancer le serveur CARLA en arrière-plan (headless) ──
# -RenderOffScreen : rendu calculé mais sans écran (les capteurs marchent).
# -nosound : pas d'audio (inutile, évite des warnings).
# -carla-rpc-port : port RPC explicite (doit matcher le client).
echo ""
echo "[1] Démarrage du serveur CARLA (headless, port $RPC_PORT)..."
cd "$CARLA_DIR"
./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=$RPC_PORT &
CARLA_PID=$!
echo "    Serveur lancé (PID $CARLA_PID)."

# Fonction de nettoyage : tuer le serveur en sortie
cleanup() {
    echo ""
    echo "[cleanup] Arrêt du serveur CARLA (PID $CARLA_PID)..."
    kill $CARLA_PID 2>/dev/null || true
}
trap cleanup EXIT

if [ "$1" == "--server-only" ]; then
    echo "    Mode --server-only : serveur lancé, en attente (Ctrl+C pour arrêter)."
    wait $CARLA_PID
    exit 0
fi

# ── 2. Attendre que le serveur soit prêt ──
echo ""
echo "[2] Attente du démarrage ($WAIT_SECONDS s)..."
for i in $(seq 1 $WAIT_SECONDS); do
    # Vérifier que le process tourne toujours
    if ! kill -0 $CARLA_PID 2>/dev/null; then
        echo "✗ Le serveur CARLA s'est arrêté prématurément. Vérifie les drivers GPU."
        echo "  Indice : 'nvidia-smi' doit montrer un GPU. CARLA 0.9.15 exige Vulkan/NVIDIA."
        exit 1
    fi
    sleep 1
    printf "."
done
echo " prêt (on l'espère)."

# ── 3. Lancer le smoke test ──
echo ""
echo "[3] Smoke test (connexion client → serveur)..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python "$SCRIPT_DIR/smoke_test.py" --port $RPC_PORT
RESULT=$?

# Le trap cleanup s'occupe d'arrêter le serveur
exit $RESULT
