#!/usr/bin/env python3
"""
validate_bridge.py — Valide le CarlaScenarioBridge sur un pod (PHASE de validation).

Le bridge a été écrit à froid et n'a jamais tourné : ce script le fait tourner pour la
première fois, avec des diagnostics riches, et compare ses résultats au KinematicRunner.

⚠️ Nécessite un pod CARLA (serveur lancé) + l'environnement chargé (source ~/rcib_env.sh).

But : vérifier que les TENDANCES sont cohérentes (passif dangereux, réactif plus sûr),
PAS que les chiffres soient identiques (la physique CARLA diffère du cinématique).

Usage (sur le pod, serveur CARLA lancé) :
    source ~/rcib_env.sh
    cd rcib-benchmark
    python3 runners/validate_bridge.py            # diagnostic + comparaison (3 seeds)
    python3 runners/validate_bridge.py --seeds 5  # plus de seeds
    python3 runners/validate_bridge.py --deep-only # juste le scénario en profondeur
"""

import sys
import os
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "rcib"))

from scenario import crossing_scenario
from kinematic_runner import run_kinematic
from ego_controller import CruiseController
from ego_planner import EgoPlanner, PassivePlanner
from metrics import evaluate
from intention.heuristic import HeuristicIntention


def _speed_profile(trace, n=6):
    """Échantillonne la vitesse de l'ego à n instants pour voir le profil."""
    if not trace.frames:
        return []
    step = max(1, len(trace.frames) // n)
    return [(round(f.t, 1), round(f.ego.speed, 1)) for f in trace.frames[::step]]


def _min_distance(trace):
    """Distance minimale ego-piéton sur toute la trace."""
    import math
    best = float("inf")
    for f in trace.frames:
        for p in f.pedestrians:
            best = min(best, math.hypot(f.ego.x - p.x, f.ego.y - p.y))
    return best


def deep_dive(run_in_carla, seed=0):
    """Exécute UN scénario passif dans CARLA avec diagnostics complets, vs cinématique."""
    spec = crossing_scenario(seed=seed)
    ped = spec.pedestrians[0]

    print("=" * 70)
    print(f"  SCÉNARIO EN PROFONDEUR — crossing seed{seed}")
    print("=" * 70)
    print(f"  ego: vitesse cible {spec.ego_target_speed} m/s, but à {spec.ego_goal_distance:.0f} m")
    print(f"  piéton: démarre t={ped.start_time:.1f}s, à x={ped.start_x:.1f} y={ped.start_y:.1f}, "
          f"vitesse {ped.speed:.2f} m/s")
    print(f"  durée {spec.duration}s, {spec.n_ticks} ticks, dt={spec.fixed_delta}s")
    print()

    # ── CARLA (passif) ──
    print("  >> Exécution dans CARLA (ego passif)...")
    passive = PassivePlanner(spec.ego_target_speed)
    ctrl = CruiseController(target_speed=spec.ego_target_speed)
    tr_carla = run_in_carla(spec, controller=ctrl, planner=passive)
    r_carla = evaluate(tr_carla)

    print(f"     frames produites : {len(tr_carla)} (attendu {spec.n_ticks})")
    print(f"     ego : départ x={tr_carla.frames[0].ego.x:.1f} -> fin x={tr_carla.frames[-1].ego.x:.1f}")
    print(f"     profil vitesse ego : {_speed_profile(tr_carla)}")
    if tr_carla.frames[0].pedestrians:
        p0 = tr_carla.frames[0].pedestrians[0]
        pN = tr_carla.frames[-1].pedestrians[0]
        print(f"     piéton : départ ({p0.x:.1f},{p0.y:.1f}) -> fin ({pN.x:.1f},{pN.y:.1f})")
    print(f"     distance min ego-piéton : {_min_distance(tr_carla):.2f} m")
    print(f"     collision : {r_carla.collision}"
          + (f" AVEC {tr_carla.frames[-1].collision_with}" if r_carla.collision else ""))
    print(f"     RCIB={r_carla.rcib_score:.3f}  min_TTC={r_carla.min_ttc:.2f}  goal={r_carla.reached_goal}")
    print()

    # ── Cinématique (passif), même scénario, pour comparer ──
    print("  >> Même scénario en cinématique (référence)...")
    tr_kin = run_kinematic(spec, planner=PassivePlanner(spec.ego_target_speed))
    r_kin = evaluate(tr_kin)
    print(f"     profil vitesse ego : {_speed_profile(tr_kin)}")
    print(f"     distance min ego-piéton : {_min_distance(tr_kin):.2f} m")
    print(f"     collision : {r_kin.collision}   RCIB={r_kin.rcib_score:.3f}  min_TTC={r_kin.min_ttc:.2f}")
    print()

    # ── Verdict ──
    print("  --- LECTURE ---")
    if len(tr_carla) != spec.n_ticks:
        print("  ⚠️  Nombre de frames inattendu : le bridge s'est peut-être arrêté tôt.")
    if tr_carla.frames[-1].ego.x - tr_carla.frames[0].ego.x < 1.0:
        print("  ⚠️  L'ego n'a quasiment pas bougé dans CARLA — vérifier le contrôle véhicule.")
    else:
        print("  ✓  L'ego se déplace dans CARLA.")
    if tr_carla.frames[0].pedestrians:
        moved = abs(pN.y - p0.y) + abs(pN.x - p0.x)
        if moved < 0.5:
            print("  ⚠️  Le piéton n'a pas bougé — vérifier WalkerControl.")
        else:
            print("  ✓  Le piéton se déplace dans CARLA.")
    if r_carla.collision and "walker" not in str(tr_carla.frames[-1].collision_with or ""):
        print(f"  ⚠️  Collision AVEC un acteur non-piéton ({tr_carla.frames[-1].collision_with}) "
              "— l'ego a peut-être quitté la route. Diagnostic à affiner.")
    print()
    return r_carla, r_kin


def comparison(run_in_carla, seeds):
    """Compare passif vs réactif, CARLA vs cinématique, sur plusieurs seeds."""
    predictor = HeuristicIntention()
    print("=" * 78)
    print(f"  COMPARAISON SUR {len(seeds)} SEEDS")
    print("=" * 78)
    print(f"  {'seed':>4} | {'exécuteur':>10} | {'planner':>8} | {'RCIB':>6} | "
          f"{'coll':>5} | {'minTTC':>6} | goal")
    print("  " + "-" * 74)

    def run_one(spec, executor, planner_kind):
        if planner_kind == "passif":
            planner = PassivePlanner(spec.ego_target_speed)
        else:
            planner = EgoPlanner(predictor, cruise_speed=spec.ego_target_speed)
        ctrl = CruiseController(target_speed=spec.ego_target_speed)
        if executor == "carla":
            tr = run_in_carla(spec, controller=ctrl, planner=planner)
        else:
            tr = run_kinematic(spec, controller=ctrl, planner=planner)
        return evaluate(tr)

    summary = {"carla_passif_coll": 0, "carla_react_coll": 0,
               "kin_passif_coll": 0, "kin_react_coll": 0}
    for seed in seeds:
        spec = crossing_scenario(seed=seed)
        for executor in ("carla", "kin"):
            for kind in ("passif", "réactif"):
                r = run_one(spec, executor, kind)
                key = f"{executor}_{'passif' if kind=='passif' else 'react'}_coll"
                summary[key] += int(r.collision)
                print(f"  {seed:>4} | {executor:>10} | {kind:>8} | {r.rcib_score:.3f} | "
                      f"{str(r.collision):>5} | {r.min_ttc:>6.2f} | {r.reached_goal}")
        print("  " + "-" * 74)

    n = len(seeds)
    print()
    print("  --- TENDANCES ---")
    print(f"  CARLA   : passif {summary['carla_passif_coll']}/{n} collisions, "
          f"réactif {summary['carla_react_coll']}/{n}")
    print(f"  Cinémat.: passif {summary['kin_passif_coll']}/{n} collisions, "
          f"réactif {summary['kin_react_coll']}/{n}")
    print()
    ok = (summary['carla_react_coll'] <= summary['carla_passif_coll'])
    if ok and summary['carla_passif_coll'] > 0:
        print("  ✓  Tendance COHÉRENTE : dans CARLA aussi, le réactif est plus sûr que le passif.")
    elif summary['carla_passif_coll'] == 0:
        print("  ⚠️  Le passif ne crée pas de conflit dans CARLA (physique différente).")
        print("      -> il faudra recalibrer le timing du scénario pour CARLA. C'est attendu.")
    else:
        print("  ⚠️  Tendance inattendue — à analyser avec les diagnostics du deep-dive.")


def main():
    ap = argparse.ArgumentParser(description="Validation du bridge CARLA")
    ap.add_argument("--seeds", type=int, default=3, help="nb de seeds pour la comparaison")
    ap.add_argument("--deep-only", action="store_true", help="seulement le scénario en profondeur")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    args = ap.parse_args()

    # Import tardif de carla (échoue proprement hors pod)
    try:
        from scenario_bridge import CarlaScenarioBridge
    except Exception as e:
        print(f"Impossible de charger le bridge CARLA : {e}")
        print("Ce script doit tourner sur un pod avec le package carla et l'env chargé.")
        sys.exit(1)

    # On réutilise UN bridge pour tous les runs (une connexion)
    def run_in_carla(spec, controller=None, planner=None):
        bridge = CarlaScenarioBridge(host=args.host, port=args.port)
        return bridge.run(spec, controller=controller, planner=planner)

    print("\n>> Connexion à CARLA et première exécution du bridge...\n")
    deep_dive(run_in_carla, seed=0)

    if not args.deep_only:
        comparison(run_in_carla, seeds=list(range(args.seeds)))

    print("\n>> Validation terminée. Éteins le pod si tu as fini.\n")


if __name__ == "__main__":
    main()
