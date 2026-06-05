#!/usr/bin/env python3
"""
Tests de la Phase 4 (Intention Adapter) — valident les baselines enrichies et la
frontière de processus (RemoteIntentionPredictor), SANS GPU ni vrai modèle SOTA.

Lancer depuis rcib/ :  python ../tests/test_intention.py
"""

import sys
import os

RCIB = os.path.join(os.path.dirname(__file__), "..", "rcib")
RUNNERS = os.path.join(os.path.dirname(__file__), "..", "runners")
sys.path.insert(0, os.path.abspath(RCIB))

from trace import AgentState
from intention.base import IntentionPredictor
from intention.heuristic import HeuristicIntention
from intention.velocity import ConstantVelocityIntention
from intention.remote import RemoteIntentionPredictor, PredictorProtocolError

SERVER = os.path.join(os.path.abspath(RUNNERS), "predictor_server_example.py")


# ── Baselines : contrat de l'interface ──

def test_all_predictors_implement_interface():
    for P in (HeuristicIntention(), ConstantVelocityIntention()):
        assert isinstance(P, IntentionPredictor)
        assert isinstance(P.name, str) and P.name

def test_scores_in_range():
    """Tout prédicteur doit renvoyer des scores dans [0,1] pour chaque piéton."""
    ego = AgentState("ego", 0, 0, 8, 0)
    peds = [AgentState("w1", 10, 2, 0, -1.5), AgentState("w2", -5, 0, 0, 0)]
    for P in (HeuristicIntention(), ConstantVelocityIntention()):
        out = P.predict_intent(ego, peds)
        assert set(out.keys()) == {"w1", "w2"}, f"{P.name}: une clé par piéton"
        for v in out.values():
            assert 0.0 <= v <= 1.0, f"{P.name}: score hors [0,1]: {v}"

def test_pedestrian_behind_is_zero():
    ego = AgentState("ego", 0, 0, 8, 0)
    behind = [AgentState("w", -10, 0, 0, 1)]
    for P in (HeuristicIntention(), ConstantVelocityIntention()):
        out = P.predict_intent(ego, behind)
        assert out["w"] == 0.0, f"{P.name}: piéton derrière doit avoir intention 0"


# ── constant-velocity : sémantique propre ──

def test_cv_high_when_crossing_soon():
    """Un piéton qui va couper la trajectoire bientôt -> intention élevée."""
    cv = ConstantVelocityIntention(horizon=3.0)
    ego = AgentState("ego", 0, 0, 8, 0)
    # piéton devant à 2m latéral, se déplaçant vers la trajectoire (vy négatif vers y=0)
    ped = AgentState("w", 12, 2, 0, -1.5)  # atteindra y=0 en ~1.3s
    out = cv.predict_intent(ego, [ped])
    assert out["w"] > 0.5, f"croisement imminent devrait donner intention élevée, eu {out['w']}"

def test_cv_low_when_moving_away():
    """Un piéton qui s'éloigne de la trajectoire -> intention faible."""
    cv = ConstantVelocityIntention()
    ego = AgentState("ego", 0, 0, 8, 0)
    ped = AgentState("w", 12, 3, 0, +1.5)  # s'éloigne (y augmente)
    out = cv.predict_intent(ego, [ped])
    assert out["w"] < 0.3, f"piéton s'éloignant devrait donner intention faible, eu {out['w']}"


# ── Frontière de processus : RemoteIntentionPredictor ──

def test_remote_matches_local():
    """Le prédicteur distant (dummy) doit reproduire la heuristique locale."""
    ego = AgentState("ego", 0, 0, 8, 0)
    peds = [AgentState("w1", 10, 2, 0, -1.5), AgentState("w2", 5, -8, 0, 0)]
    local = HeuristicIntention().predict_intent(ego, peds)
    remote = RemoteIntentionPredictor(["python3", SERVER])
    try:
        out = remote.predict_intent(ego, peds)
    finally:
        remote.close()
    for k in local:
        assert abs(out[k] - local[k]) < 0.01, f"distant != local sur {k}"

def test_remote_reused_across_calls():
    """Le processus distant est réutilisé (un seul démarrage), pas relancé par appel."""
    ego = AgentState("ego", 0, 0, 8, 0)
    peds = [AgentState("w1", 10, 2, 0, -1.5)]
    remote = RemoteIntentionPredictor(["python3", SERVER])
    try:
        a = remote.predict_intent(ego, peds)
        pid1 = remote.proc.pid
        b = remote.predict_intent(ego, peds)
        pid2 = remote.proc.pid
    finally:
        remote.close()
    assert a == b, "appels identiques -> résultats identiques"
    assert pid1 == pid2, "le processus doit être réutilisé (même PID)"

def test_remote_scores_in_range():
    ego = AgentState("ego", 0, 0, 8, 0)
    peds = [AgentState("w1", 10, 2, 0, -1.5), AgentState("w2", -5, 0, 0, 0)]
    remote = RemoteIntentionPredictor(["python3", SERVER])
    try:
        out = remote.predict_intent(ego, peds)
    finally:
        remote.close()
    assert set(out.keys()) == {"w1", "w2"}
    for v in out.values():
        assert 0.0 <= v <= 1.0

def test_remote_bad_command_raises():
    """Une commande introuvable doit lever une erreur de protocole claire."""
    remote = RemoteIntentionPredictor(["this_binary_does_not_exist_xyz"])
    ego = AgentState("ego", 0, 0, 8, 0)
    try:
        raised = False
        try:
            remote.predict_intent(ego, [])
        except PredictorProtocolError:
            raised = True
        assert raised, "un binaire introuvable devrait lever PredictorProtocolError"
    finally:
        remote.close()

def test_remote_context_manager():
    """Le with-statement ferme proprement le processus."""
    ego = AgentState("ego", 0, 0, 8, 0)
    peds = [AgentState("w1", 10, 2, 0, -1.5)]
    with RemoteIntentionPredictor(["python3", SERVER]) as remote:
        out = remote.predict_intent(ego, peds)
        assert "w1" in out
    # après le with, le process doit être fermé
    assert remote.proc is None, "le processus doit être fermé en sortie de with"


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
    print("=== Tests de la Phase 4 (Intention Adapter + frontière de processus) ===")
    ok = _run_all()
    sys.exit(0 if ok else 1)
