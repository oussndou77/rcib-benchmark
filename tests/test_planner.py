#!/usr/bin/env python3
"""
Tests de la Phase 3 (Ego Planner réactif / closed-loop) — validés SANS CARLA via
le KinematicRunner. Verrouillent les propriétés essentielles du planner :
  - le réactif évite les collisions que le passif subit
  - le réactif atteint quand même son but
  - le réactif score nettement mieux que le passif au sens RCIB
  - RCIB ordonne correctement passif < agressif(dangereux) et passif < prudent

Lancer depuis rcib/ :  python ../tests/test_planner.py
"""

import sys
import os

RCIB = os.path.join(os.path.dirname(__file__), "..", "rcib")
sys.path.insert(0, os.path.abspath(RCIB))

from scenario import crossing_scenario, no_pedestrian_scenario
from kinematic_runner import run_kinematic
from ego_controller import CruiseController
from ego_planner import EgoPlanner, PassivePlanner, PlannerConfig
from metrics import evaluate
from intention.heuristic import HeuristicIntention
from trace import AgentState

PRED = HeuristicIntention()


def _reactive(spec, config=None):
    planner = EgoPlanner(PRED, cruise_speed=spec.ego_target_speed, config=config)
    ctrl = CruiseController(target_speed=spec.ego_target_speed)
    return evaluate(run_kinematic(spec, controller=ctrl, planner=planner))

def _passive(spec):
    return evaluate(run_kinematic(spec, planner=PassivePlanner(spec.ego_target_speed)))


# ── Propriétés de base du planner ──

def test_planner_is_callable():
    p = EgoPlanner(PRED, cruise_speed=8.0)
    ego = AgentState("ego", 0, 0, 8, 0)
    out = p(ego, [], 0.0)
    assert out == 8.0, "sans piéton, le planner garde la vitesse de croisière"

def test_planner_slows_for_pedestrian_ahead():
    p = EgoPlanner(PRED, cruise_speed=8.0)
    ego = AgentState("ego", 0, 0, 8, 0)
    # piéton droit devant, proche, sur la trajectoire
    ped = AgentState("walker_01", 10, 0, 0, 1.5)
    out = p(ego, [ped], 0.0)
    assert out < 8.0, "un piéton menaçant devant doit faire ralentir"

def test_planner_ignores_pedestrian_behind():
    p = EgoPlanner(PRED, cruise_speed=8.0)
    ego = AgentState("ego", 20, 0, 8, 0)
    ped = AgentState("walker_01", 5, 0, 0, 1.5)   # derrière l'ego
    out = p(ego, [ped], 0.0)
    assert out == 8.0, "un piéton derrière l'ego ne doit pas le ralentir"

def test_passive_planner_constant():
    p = PassivePlanner(cruise_speed=8.0)
    ego = AgentState("ego", 0, 0, 5, 0)
    ped = AgentState("walker_01", 10, 0, 0, 1.5)
    assert p(ego, [ped], 0.0) == 8.0, "le passif ignore les piétons"


# ── Closed-loop : sécurité, avancement, score ──

def test_reactive_avoids_collisions():
    """Sur le banc de scénarios, le réactif (défaut prudent) évite TOUTES les collisions."""
    collisions = sum(_reactive(crossing_scenario(seed=s)).collision for s in range(12))
    assert collisions == 0, f"le réactif devrait éviter les collisions, {collisions}/12 subies"

def test_reactive_reaches_goal():
    """Le réactif ne se contente pas de freiner : il atteint son but."""
    goals = sum(_reactive(crossing_scenario(seed=s)).reached_goal for s in range(12))
    assert goals >= 11, f"le réactif devrait atteindre le but, {goals}/12 seulement"

def test_reactive_beats_passive():
    """Le score RCIB du réactif domine nettement celui du passif."""
    N = 12
    passive = sum(_passive(crossing_scenario(seed=s)).rcib_score for s in range(N)) / N
    reactive = sum(_reactive(crossing_scenario(seed=s)).rcib_score for s in range(N)) / N
    assert reactive > passive + 0.3, f"réactif {reactive:.2f} doit battre passif {passive:.2f}"

def test_passive_always_collides():
    """Le scénario est bien un piège : le passif entre toujours en collision."""
    collisions = sum(_passive(crossing_scenario(seed=s)).collision for s in range(12))
    assert collisions == 12, f"le passif devrait toujours percuter, {collisions}/12"


# ── RCIB ordonne les comportements ──

def test_aggressive_worse_than_prudent():
    """Un planner trop agressif (freine tard) score moins bien qu'un prudent."""
    N = 8
    prudent_cfg = PlannerConfig()  # défaut = prudent
    aggressive_cfg = PlannerConfig(intent_threshold=0.35, safety_margin=2.0,
                                   comfort_decel=3.0, emergency_decel=7.0)
    prudent = sum(_reactive(crossing_scenario(seed=s), prudent_cfg).rcib_score
                  for s in range(N)) / N
    aggressive = sum(_reactive(crossing_scenario(seed=s), aggressive_cfg).rcib_score
                     for s in range(N)) / N
    assert prudent > aggressive, \
        f"le prudent {prudent:.2f} doit dominer l'agressif {aggressive:.2f}"


def test_free_road_unaffected_by_planner():
    """Sur route libre, le planner réactif ne dégrade pas la conduite."""
    r = _reactive(no_pedestrian_scenario(seed=0))
    assert not r.collision and r.reached_goal
    assert r.rcib_score > 0.9, "route libre : le réactif doit rester quasi parfait"


def _run_all():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    passed = 0
    for t in tests:
        try:
            t()
            print(f"  ✓ {t.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"  ✗ {t.__name__} — ÉCHEC: {e}")
        except Exception as e:
            print(f"  ✗ {t.__name__} — ERREUR: {type(e).__name__}: {e}")
    print(f"\n{passed}/{len(tests)} tests passés")
    return passed == len(tests)


if __name__ == "__main__":
    print("=== Tests de la Phase 3 (Ego Planner réactif) ===")
    ok = _run_all()
    sys.exit(0 if ok else 1)
