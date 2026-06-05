#!/usr/bin/env python3
"""
Tests de la Phase 5 (Batch runner + leaderboard) — valident l'agrégation par modèle,
le classement, la reproductibilité et la sauvegarde, SANS GPU.

Lancer depuis rcib/ :  python ../tests/test_benchmark.py
"""

import sys
import os
import tempfile

RCIB = os.path.join(os.path.dirname(__file__), "..", "rcib")
sys.path.insert(0, os.path.abspath(RCIB))

from benchmark import (run_benchmark, evaluate_model, default_suite,
                       format_leaderboard, save_leaderboard, ModelResult)
from intention.heuristic import HeuristicIntention
from intention.velocity import ConstantVelocityIntention


def test_default_suite_reproducible():
    a = default_suite(n_crossing=5, n_free=2)
    b = default_suite(n_crossing=5, n_free=2)
    assert len(a) == 7
    assert [s.to_dict() for s in a] == [s.to_dict() for s in b]

def test_evaluate_model_returns_aggregate():
    suite = default_suite(n_crossing=4, n_free=1)
    res = evaluate_model(HeuristicIntention(), suite)
    assert isinstance(res, ModelResult)
    assert res.n_scenarios == 5
    assert 0.0 <= res.rcib_mean <= 1.0
    assert 0.0 <= res.collision_rate <= 1.0
    assert 0.0 <= res.goal_rate <= 1.0
    assert len(res.per_scenario) == 5

def test_benchmark_ranks_by_rcib():
    suite = default_suite(n_crossing=8, n_free=2)
    results = run_benchmark([HeuristicIntention(), ConstantVelocityIntention()],
                            suite=suite)
    # le leaderboard doit être trié par RCIB décroissant
    assert results[0].rcib_mean >= results[1].rcib_mean

def test_benchmark_reproducible():
    """Deux exécutions de la même suite donnent un classement identique."""
    suite = default_suite(n_crossing=6, n_free=2)
    preds1 = [HeuristicIntention(), ConstantVelocityIntention()]
    preds2 = [HeuristicIntention(), ConstantVelocityIntention()]
    r1 = run_benchmark(preds1, suite=suite)
    r2 = run_benchmark(preds2, suite=suite)
    assert [r.model_name for r in r1] == [r.model_name for r in r2]
    for a, b in zip(r1, r2):
        assert abs(a.rcib_mean - b.rcib_mean) < 1e-9

def test_reactive_beats_nothing_in_benchmark():
    """Sur la suite, les prédicteurs réactifs évitent les collisions (collision_rate=0)."""
    suite = default_suite(n_crossing=12, n_free=0)
    results = run_benchmark([HeuristicIntention(), ConstantVelocityIntention()],
                            suite=suite)
    for r in results:
        assert r.collision_rate == 0.0, f"{r.model_name} devrait éviter les collisions"
        assert r.goal_rate == 1.0, f"{r.model_name} devrait atteindre tous les buts"

def test_leaderboard_formatting():
    suite = default_suite(n_crossing=3, n_free=1)
    results = run_benchmark([HeuristicIntention()], suite=suite)
    txt = format_leaderboard(results)
    assert "RCIB" in txt and "heuristic_v0" in txt

def test_save_and_structure():
    suite = default_suite(n_crossing=3, n_free=1)
    results = run_benchmark([HeuristicIntention(), ConstantVelocityIntention()],
                            suite=suite)
    with tempfile.TemporaryDirectory() as d:
        path = save_leaderboard(results, os.path.join(d, "lb.json"))
        assert os.path.exists(path)
        import json
        with open(path) as f:
            lb = json.load(f)
        assert lb["n_models"] == 2
        assert lb["n_scenarios"] == 4
        assert len(lb["ranking"]) == 2
        # chaque entrée a le détail par scénario
        assert len(lb["ranking"][0]["per_scenario"]) == 4

def test_std_zero_for_single_scenario():
    suite = default_suite(n_crossing=1, n_free=0)
    res = evaluate_model(HeuristicIntention(), suite)
    assert res.rcib_std == 0.0


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
    print("=== Tests de la Phase 5 (Batch runner + leaderboard) ===")
    ok = _run_all()
    sys.exit(0 if ok else 1)
