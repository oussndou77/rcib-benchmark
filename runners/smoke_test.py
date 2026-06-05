#!/usr/bin/env python3
"""
smoke_test.py — Phase 0 : vérifie que le client Python parle bien au serveur CARLA.

Objectif minimal et précis :
  1. se connecter au serveur CARLA (localhost:2000)
  2. VÉRIFIER que la version client == version serveur (le piège n°1)
  3. charger une carte légère et faire tourner quelques ticks en mode synchrone
  4. spawn un véhicule (ego) et confirmer qu'il apparaît

Si ce test passe, l'infrastructure GPU + CARLA + client est bonne, et on peut
construire le Scenario Bridge (Phase 2) dessus en confiance.

Usage (dans le pod, serveur CARLA déjà lancé) :
    python smoke_test.py
    python smoke_test.py --host localhost --port 2000 --map Town01 --ticks 20
"""

import sys
import time
import argparse


def main():
    parser = argparse.ArgumentParser(description="Smoke test CARLA (Phase 0)")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--map", default="Town01", help="carte légère recommandée")
    parser.add_argument("--ticks", type=int, default=20, help="nombre de ticks synchrones")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="timeout connexion (s) — CARLA met du temps à démarrer")
    args = parser.parse_args()

    # ── 0. Import du package carla ──
    try:
        import carla
    except ImportError as e:
        print(f"✗ ÉCHEC : le package Python 'carla' n'est pas importable ({e}).")
        print("  → As-tu lancé le setup et chargé l'environnement ?")
        print("      bash setup_runpod.sh && source ~/rcib_env.sh")
        print("  → Si une lib système manque (libXXX.so), télécharge-la sans root :")
        print("      cd ~/debs && apt-get download <paquet> && dpkg -x <deb> ~/locallibs")
        return 1
    print("✓ package carla importé.")

    # ── 1. Connexion ──
    print(f"\n[1/4] Connexion à {args.host}:{args.port} (timeout {args.timeout}s)...")
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    try:
        server_version = client.get_server_version()
        client_version = client.get_client_version()
    except RuntimeError as e:
        print(f"✗ ÉCHEC connexion : {e}")
        print("  → Le serveur CARLA est-il lancé ? (voir launch_carla.sh)")
        print("  → Attends ~30-60s après le lancement du serveur avant ce test.")
        return 1

    print(f"  client_version = {client_version}")
    print(f"  server_version = {server_version}")

    # ── 2. LE CHECK CRITIQUE : versions identiques (piège n°1) ──
    if client_version != server_version:
        print(f"\n✗ ÉCHEC : MISMATCH DE VERSION client≠serveur.")
        print(f"  C'est le piège n°1. Le package Python carla DOIT correspondre au serveur.")
        print(f"  → Réinstalle le wheel depuis l'image : "
              f"pip install /home/carla/PythonAPI/carla/dist/carla-{server_version}-*.whl")
        return 1
    print(f"✓ [2/4] Versions identiques ({server_version}) — pas de mismatch RPC.")

    # ── 3. Charger une carte + mode synchrone + ticks ──
    print(f"\n[3/4] Chargement de la carte {args.map} + {args.ticks} ticks synchrones...")
    try:
        world = client.load_world(args.map)
    except RuntimeError as e:
        print(f"  (load_world a échoué, on tente get_world : {e})")
        world = client.get_world()

    # Mode synchrone déterministe (essentiel pour la reproductibilité, cf. PLAN §7)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05   # 20 Hz
    world.apply_settings(settings)

    try:
        for i in range(args.ticks):
            world.tick()
        print(f"✓ {args.ticks} ticks exécutés en mode synchrone (20 Hz).")
    finally:
        # Toujours restaurer le mode asynchrone en sortie (sinon le serveur reste bloqué)
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

    # ── 4. Spawn d'un véhicule (ego) ──
    print(f"\n[4/4] Spawn d'un véhicule de test...")
    bp_lib = world.get_blueprint_library()
    vehicle_bp = bp_lib.filter("vehicle.*")[0]
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print("✗ ÉCHEC : aucun point de spawn sur cette carte.")
        return 1

    vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[0])
    if vehicle is None:
        print("✗ ÉCHEC : impossible de spawn le véhicule.")
        return 1
    loc = vehicle.get_location()
    print(f"✓ Véhicule '{vehicle_bp.id}' spawné à (x={loc.x:.1f}, y={loc.y:.1f}, z={loc.z:.1f})")

    # Nettoyage
    vehicle.destroy()
    print("✓ Véhicule détruit (nettoyage).")

    # ── Verdict ──
    print(f"\n{'='*56}")
    print("✓✓✓ SMOKE TEST RÉUSSI — CARLA + client + GPU opérationnels.")
    print(f"    Version verrouillée : {server_version}")
    print("    Prêt pour la Phase 2 (Scenario Bridge).")
    print(f"{'='*56}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
