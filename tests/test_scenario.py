#!/usr/bin/env python3
"""
Tests de la Phase 2 (Scenario Bridge) — valident la logique SANS CARLA.
Le KinematicRunner permet de tout tester à froid ; le bridge CARLA (scenario_bridge.py)
n'est validable que sur pod, mais on vérifie ici qu'il s'importe et échoue proprement
sans le package carla.

Lancer depuis rcib/ :  python ../tests/test_scenario.py
"""

import sys
import os

RCIB = os.path.join(os.path.dirname(__file__), "..", "rcib")
sys.path.insert(0, os.path.abspath(RCIB))

from scenario import crossing_scenario, no_pedestrian_scenario, PedestrianSpec
from ego_controller import CruiseController
from kinematic_runner import run_kinematic
from metrics import evaluate
from trace import Trace


# ── Scénario : reproductibilité ──

def test_scenario_reproducible():
    a = crossing_scenario(seed=42).to_dict()
    b = crossing_scenario(seed=42).to_dict()
    assert a == b, "même seed doit donner le même scénario"

def test_scenario_seeds_differ():
    a = crossing_scenario(seed=1).to_dict()
    b = crossing_scenario(seed=2).to_dict()
    assert a != b, "seeds différentes doivent différer"


# ── PedestrianSpec : cinématique ──

def test_pedestrian_stationary_before_start():
    p = PedestrianSpec("p", start_x=10, start_y=5, walk_direction=(0, 1),
                       speed=2.0, start_time=3.0)
    # avant start_time : immobile
    assert p.position_at(1.0) == (10, 5)
    assert p.velocity_at(1.0) == (0.0, 0.0)
    # après : il a bougé
    px, py = p.position_at(4.0)   # 1s de marche à 2 m/s vers +y
    assert abs(px - 10) < 1e-6 and abs(py - 7) < 1e-6

def test_pedestrian_position_trigger():
    """Le déclenchement par position : le piéton part quand l'ego s'approche du croisement."""
    p = PedestrianSpec("p", start_x=40, start_y=-3, walk_direction=(0, 1),
                       speed=1.5, trigger_distance=15.0)
    # ego loin (à 0) : le croisement est à 40, gap=40 > 15 -> pas encore
    assert not p.should_start(t=5.0, ego_longitudinal=0.0)
    # ego à 30 : gap = 40-30 = 10 <= 15 -> démarre
    assert p.should_start(t=5.0, ego_longitudinal=30.0)
    # ego a dépassé le croisement (à 45) : gap négatif -> ne démarre pas (déjà passé)
    assert not p.should_start(t=5.0, ego_longitudinal=45.0)

def test_crossing_uses_position_trigger():
    """Le scénario de croisement par défaut utilise le déclenchement par position."""
    spec = crossing_scenario(seed=0)
    assert spec.pedestrians[0].trigger_distance is not None, \
        "le scénario de croisement doit déclencher par position (robuste à l'accélération)"


def test_time_trigger_still_works():
    """Le mode temps (start_time) reste fonctionnel pour la rétrocompatibilité."""
    p = PedestrianSpec("p", start_x=10, start_y=0, walk_direction=(0, 1),
                       speed=1.0, start_time=3.0)  # pas de trigger_distance
    assert not p.should_start(t=1.0, ego_longitudinal=999)
    assert p.should_start(t=3.5, ego_longitudinal=0)


# ── CruiseController ──

def test_cruise_accelerates_when_slow():
    c = CruiseController(target_speed=10.0)
    cmd = c.control(current_speed=0.0)
    assert cmd.throttle > 0 and cmd.brake == 0

def test_cruise_brakes_when_fast():
    c = CruiseController(target_speed=5.0)
    cmd = c.control(current_speed=10.0)
    assert cmd.brake > 0 and cmd.throttle == 0

def test_cruise_target_zero_brakes_hard():
    c = CruiseController(target_speed=0.0)
    cmd = c.control(current_speed=5.0)
    assert cmd.brake == 1.0


# ── KinematicRunner + métriques ──

def test_free_road_reaches_goal():
    tr = run_kinematic(no_pedestrian_scenario(seed=0))
    r = evaluate(tr)
    assert not r.collision
    assert r.reached_goal
    assert r.rcib_score > 0.9

def test_crossing_passive_is_dangerous():
    """Le scénario de croisement DOIT créer un conflit avec un ego passif."""
    # Sur plusieurs seeds, l'ego passif doit majoritairement entrer en collision.
    collisions = 0
    for seed in range(10):
        tr = run_kinematic(crossing_scenario(seed=seed))
        if evaluate(tr).collision:
            collisions += 1
    assert collisions >= 8, f"le scénario devrait être dangereux, {collisions}/10 collisions"

def test_runner_produces_valid_trace():
    tr = run_kinematic(crossing_scenario(seed=0))
    assert isinstance(tr, Trace)
    assert len(tr) == crossing_scenario(seed=0).n_ticks
    # round-trip JSON (sérialisation/désérialisation)
    d = tr.to_dict()
    tr2 = Trace.from_dict(d)
    assert len(tr2) == len(tr)
    assert tr2.scenario_id == tr.scenario_id


# ── Bridge CARLA : import sans carla doit échouer proprement ──

def test_bridge_imports_without_carla():
    """Le module bridge s'importe même sans carla ; l'erreur n'apparaît qu'à l'instanciation."""
    import scenario_bridge  # ne doit PAS planter à l'import (carla importé localement)
    assert hasattr(scenario_bridge, "run_in_carla")
    assert hasattr(scenario_bridge, "CarlaScenarioBridge")


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
    print("=== Tests de la Phase 2 (Scenario Bridge) ===")
    ok = _run_all()
    sys.exit(0 if ok else 1)
