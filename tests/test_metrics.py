#!/usr/bin/env python3
"""
Tests du Metrics Harness — valident la physique et les scores SANS GPU/CARLA.
Lancer depuis le dossier rcib/ :  python -m pytest ../tests/test_metrics.py -v
ou directement :                   python ../tests/test_metrics.py
"""

import sys
import os
import math

# Rendre le package rcib importable (imports plats)
RCIB = os.path.join(os.path.dirname(__file__), "..", "rcib")
sys.path.insert(0, os.path.abspath(RCIB))

from trace import Trace, Frame, AgentState
from metrics import evaluate, instantaneous_ttc, MetricsConfig
from intention.heuristic import HeuristicIntention


def _straight_run(n=51, dt=0.1, v=10.0, pedestrians_fn=None, goal=(50, 0)):
    """Construit une trace d'ego roulant droit ; pedestrians_fn(t) -> [AgentState]."""
    frames = []
    x = 0.0
    for i in range(n):
        t = i * dt
        x = v * t
        peds = pedestrians_fn(t) if pedestrians_fn else []
        frames.append(Frame(t=t, ego=AgentState("ego", x=x, y=0, vx=v, vy=0),
                            pedestrians=peds))
    return Trace(frames=frames, ego_goal=goal, goal_radius=3.0)


# ── TTC (physique) ──

def test_ttc_frontal_exact():
    cfg = MetricsConfig()
    ego = AgentState("ego", 0, 0, 10, 0)
    ped = AgentState("p", 20, 0, 0, 0)
    # collision à distance R=1.5 → 18.5m à 10m/s → 1.85s
    assert abs(instantaneous_ttc(ego, ped, cfg.collision_radius, cfg.ttc_cap) - 1.85) < 0.01

def test_ttc_diverging_is_cap():
    cfg = MetricsConfig()
    ego = AgentState("ego", 0, 0, 10, 0)
    ped = AgentState("p", 20, 0, 15, 0)  # fuit plus vite que l'ego
    assert instantaneous_ttc(ego, ped, cfg.collision_radius, cfg.ttc_cap) == cfg.ttc_cap

def test_ttc_already_in_contact():
    cfg = MetricsConfig()
    ego = AgentState("ego", 0, 0, 10, 0)
    ped = AgentState("p", 1.0, 0, 0, 0)  # déjà à <1.5m
    assert instantaneous_ttc(ego, ped, cfg.collision_radius, cfg.ttc_cap) == 0.0


# ── Harness complet ──

def test_free_road():
    r = evaluate(_straight_run())
    assert not r.collision
    assert r.reached_goal
    assert r.rcib_score > 0.9

def test_frontal_collision():
    r = evaluate(_straight_run(pedestrians_fn=lambda t: [AgentState("p", 25, 0, 0, 0)]))
    assert r.collision
    assert r.collision_speed > 9
    assert r.safety_score == 0.0
    assert r.rcib_score < 0.35

def test_reactive_beats_passive():
    """Le test fondateur : un ego qui freine doit scorer mieux qu'un ego qui fonce."""
    dt = 0.1
    def run(reactive):
        frames = []
        v, x = 10.0, 0.0
        for i in range(61):
            t = i * dt
            if reactive and t >= 0.5 and v > 0:
                v = max(0.0, v - 5.0 * dt)
            x += v * dt
            frames.append(Frame(t=t, ego=AgentState("ego", x, 0, v, 0),
                                pedestrians=[AgentState("p", 30, 0, 0, 0)]))
        return Trace(frames=frames, ego_goal=(30, 0), goal_radius=6.0)
    passive = evaluate(run(False))
    reactive = evaluate(run(True))
    assert passive.collision
    assert not reactive.collision
    assert reactive.rcib_score > passive.rcib_score

def test_hard_braking_detected():
    dt = 0.1
    frames = []
    v, x = 10.0, 0.0
    for i in range(61):
        t = i * dt
        if t >= 0.5 and v > 0:
            v = max(0.0, v - 5.0 * dt)   # 5 m/s² > seuil 3.9
        x += v * dt
        frames.append(Frame(t=t, ego=AgentState("ego", x, 0, v, 0)))
    r = evaluate(Trace(frames=frames, ego_goal=(20, 0), goal_radius=99))
    assert r.max_decel > 3.9
    assert r.n_hard_brakes > 0
    assert r.comfort_score < 1.0

def test_degenerate_trace():
    r = evaluate(Trace(frames=[], ego_goal=(10, 0)))
    assert r.rcib_score == 0.0


# ── Baseline d'intention ──

def test_heuristic_ordering():
    h = HeuristicIntention()
    ego = AgentState("ego", 0, 0, 10, 0)
    crossing = AgentState("crossing", 20, 5, 0, -3)
    far = AgentState("far", 20, 8, 0, 0)
    behind = AgentState("behind", -10, 0, 0, 0)
    s = h.predict_intent(ego, [crossing, far, behind])
    assert s["behind"] == 0.0
    assert s["crossing"] > s["far"]


def _run_all():
    """Exécute tous les tests sans pytest."""
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
    print("=== Tests du Metrics Harness ===")
    ok = _run_all()
    sys.exit(0 if ok else 1)
