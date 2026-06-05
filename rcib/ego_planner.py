#!/usr/bin/env python3
"""
rcib.ego_planner — Ego Planner réactif (PUR Python, testable sans GPU).

C'est le cœur du closed-loop de RCIB : à chaque tick, il décide une VITESSE CIBLE
pour l'ego en fonction du risque que représentent les piétons. Le cruise control
(ego_controller) traduit ensuite cette cible en accélérateur/frein. L'action change
l'état du monde au tick suivant → boucle fermée.

Principe de décision — un piéton n'impose un ralentissement que s'il est À LA FOIS :
  - intentionnel (score de l'IntentionPredictor élevé), ET
  - cinématiquement menaçant (proche sur la trajectoire, TTC court)

Le planner raisonne en DISTANCE D'ARRÊT : pour s'arrêter avant un piéton qui coupe
la route, il faut commencer à ralentir assez tôt. Plutôt qu'un seuil binaire, il
calcule une vitesse sûre continue, pondérée par l'intention. Résultat : freinage
anticipé et gradué (bon pour le confort) plutôt que freinage d'urgence (mauvais).

Le planner est PARAMÉTRABLE (prudence) → RCIB peut comparer différents réglages.
Il accepte n'importe quel IntentionPredictor via l'interface stable → la Phase 4
(PIEPredict, Trajectron++) se branche sans rien changer ici.
"""

import math
from dataclasses import dataclass
from typing import List

from trace import AgentState
from intention.base import IntentionPredictor


@dataclass
class PlannerConfig:
    """Paramètres du planner — règlent le compromis sécurité/confort/avancement.

    Les valeurs par défaut sont calibrées 'prudent' : avec un prédicteur d'intention
    faible (baseline heuristique), freiner tôt et large compense la détection tardive
    et donne le meilleur score RCIB. Un meilleur prédicteur (Phase 4) permettrait des
    réglages moins prudents (moins de marge) sans perdre en sécurité — c'est ce que
    RCIB est fait pour mesurer.
    """
    intent_threshold: float = 0.10   # en-dessous, on ignore le piéton
    safety_margin: float = 8.0       # m : marge de distance qu'on veut garder
    comfort_decel: float = 1.0       # m/s² : décélération douce visée (anticipation)
    emergency_decel: float = 6.0     # m/s² : décélération forte autorisée si urgence
    detection_range: float = 45.0    # m : au-delà, on ne considère pas le piéton
    corridor_halfwidth: float = 3.0  # m : largeur du couloir devant l'ego
    lateral_consider: float = 8.0    # m : écart latéral max pour considérer un piéton qui approche


class EgoPlanner:
    """
    Décide la vitesse cible de l'ego à chaque tick, à partir des piétons et de
    leur intention prédite. Utilisé comme `planner` par le KinematicRunner et le
    CarlaScenarioBridge : planner(ego, peds, t) -> target_speed.
    """

    def __init__(self, predictor: IntentionPredictor,
                 cruise_speed: float,
                 config: PlannerConfig = None):
        self.predictor = predictor
        self.cruise_speed = cruise_speed
        self.cfg = config or PlannerConfig()
        self.name = f"reactive[{predictor.name}]"

    def __call__(self, ego: AgentState, pedestrians: List[AgentState],
                 t: float) -> float:
        """Retourne la vitesse cible (m/s) pour ce tick."""
        if not pedestrians:
            return self.cruise_speed

        intents = self.predictor.predict_intent(ego, pedestrians)

        # Direction de l'ego (unitaire). Si ~arrêté, on regarde vers +x par défaut.
        speed = ego.speed
        if speed < 0.1:
            dx, dy = 1.0, 0.0
        else:
            dx, dy = ego.vx / speed, ego.vy / speed
        nx, ny = -dy, dx   # normale (latéral)

        # Pour chaque piéton menaçant, calculer la vitesse sûre, et prendre la plus basse
        safe_speed = self.cruise_speed
        for ped in pedestrians:
            intent = intents.get(ped.id, 0.0)
            if intent < self.cfg.intent_threshold:
                continue

            # Géométrie relative
            rx, ry = ped.x - ego.x, ped.y - ego.y
            longitudinal = rx * dx + ry * dy        # distance devant (>0)
            lateral = abs(rx * nx + ry * ny)         # écart latéral

            # Ignorer : derrière l'ego, ou hors de portée longitudinale
            if longitudinal <= 0 or longitudinal > self.cfg.detection_range:
                continue
            # Ignorer un piéton latéralement très éloigné
            if lateral > self.cfg.lateral_consider:
                continue

            # Le piéton est-il dans le couloir, ou s'en approche-t-il ?
            in_corridor = lateral <= self.cfg.corridor_halfwidth
            threat = intent if in_corridor else intent * 0.6

            # ── (a) Anticipation : ralentissement doux et progressif ──
            # Vitesse permettant de s'arrêter en douceur avant le piéton.
            stopping_distance = max(0.1, longitudinal - self.cfg.safety_margin)
            v_smooth = math.sqrt(2.0 * self.cfg.comfort_decel * stopping_distance)
            v_target = threat * v_smooth + (1.0 - threat) * self.cruise_speed
            safe_speed = min(safe_speed, v_target)

            # ── (b) Sécurité : override si la collision devient imminente ──
            # Si le piéton est DANS le couloir et proche, on calcule la vitesse max
            # autorisant un arrêt même à décélération forte (pas juste douce).
            # Ça prime sur le confort : mieux vaut un freinage sec qu'une collision.
            if in_corridor and intent >= 0.3:
                # distance brute (marge réduite pour l'urgence)
                d_emergency = max(0.1, longitudinal - 1.5)
                v_emergency = math.sqrt(2.0 * self.cfg.emergency_decel * d_emergency)
                safe_speed = min(safe_speed, v_emergency)

        return max(0.0, min(self.cruise_speed, safe_speed))


class PassivePlanner:
    """Planner de référence : ignore les piétons, roule à vitesse constante.
    C'est la baseline 'sans réaction' que tout planner réactif doit battre."""

    def __init__(self, cruise_speed: float):
        self.cruise_speed = cruise_speed
        self.name = "passive"

    def __call__(self, ego: AgentState, pedestrians: List[AgentState],
                 t: float) -> float:
        return self.cruise_speed
