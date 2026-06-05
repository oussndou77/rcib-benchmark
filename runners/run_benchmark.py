#!/usr/bin/env python3
"""
run_benchmark.py — Lance le benchmark RCIB complet et produit le leaderboard.

Point d'entrée de la Phase 5. Fait tourner les prédicteurs sur la grille de scénarios
et affiche/sauvegarde le classement.

Usage :
    # À froid (sans GPU), exécuteur cinématique — pour le dev et la CI :
    python3 runners/run_benchmark.py

    # Sur un pod CARLA (physique réelle) :
    python3 runners/run_benchmark.py --carla

    # Personnaliser la grille :
    python3 runners/run_benchmark.py --crossing 20 --free 5 --save results/lb.json

Pour ajouter un prédicteur (ex. un modèle isolé via la frontière de processus),
voir docs/PLUGGING_A_MODEL.md et éditer build_predictors() ci-dessous.
"""

import sys
import os
import argparse

# Permettre l'import du package rcib quelle que soit l'origine de l'appel
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "rcib"))

from benchmark import (run_benchmark, default_suite, format_leaderboard,
                       save_leaderboard)
from intention.heuristic import HeuristicIntention
from intention.velocity import ConstantVelocityIntention


def build_predictors(include_remote: bool = False):
    """
    Construit la liste des prédicteurs à comparer.
    Pour brancher un vrai modèle SOTA isolé, ajouter ici un RemoteIntentionPredictor
    (voir docs/PLUGGING_A_MODEL.md), par exemple :

        from intention.remote import RemoteIntentionPredictor
        predictors.append(RemoteIntentionPredictor(
            ["python3.5", "/chemin/pie_server.py"], name="PIEPredict"))
    """
    predictors = [
        HeuristicIntention(),
        ConstantVelocityIntention(),
    ]
    if include_remote:
        from intention.remote import RemoteIntentionPredictor
        server = os.path.join(HERE, "predictor_server_example.py")
        predictors.append(RemoteIntentionPredictor(
            ["python3", server], name="remote_example"))
    return predictors


def main():
    ap = argparse.ArgumentParser(description="Benchmark RCIB — leaderboard de prédicteurs d'intention")
    ap.add_argument("--carla", action="store_true",
                    help="utiliser l'exécuteur CARLA (sur pod) au lieu du cinématique")
    ap.add_argument("--crossing", type=int, default=12, help="nb de scénarios de traversée")
    ap.add_argument("--free", type=int, default=3, help="nb de scénarios route libre")
    ap.add_argument("--remote", action="store_true",
                    help="inclure le prédicteur d'exemple via la frontière de processus")
    ap.add_argument("--save", type=str, default="results/leaderboard.json",
                    help="chemin de sauvegarde du leaderboard JSON")
    args = ap.parse_args()

    # Choisir l'exécuteur
    if args.carla:
        from scenario_bridge import run_in_carla
        runner = run_in_carla
        print(">> Exécuteur : CARLA (physique réelle)")
    else:
        from kinematic_runner import run_kinematic
        runner = run_kinematic
        print(">> Exécuteur : cinématique (sans GPU)")

    suite = default_suite(n_crossing=args.crossing, n_free=args.free)
    predictors = build_predictors(include_remote=args.remote)

    print(f">> {len(predictors)} prédicteurs sur {len(suite)} scénarios "
          f"({args.crossing} traversées + {args.free} routes libres)\n")

    results = run_benchmark(predictors, suite=suite, runner=runner)

    print(format_leaderboard(results))

    # Fermer proprement d'éventuels prédicteurs distants
    for p in predictors:
        if hasattr(p, "close"):
            p.close()

    path = save_leaderboard(results, args.save)
    print(f"\nLeaderboard sauvegardé : {path}")


if __name__ == "__main__":
    main()
