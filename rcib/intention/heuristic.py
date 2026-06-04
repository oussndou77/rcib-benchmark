#!/usr/bin/env python3
"""
rcib.intention.heuristic — Baseline d'intention V0 (heuristique pure Python).

Aucune dépendance externe (ni TF, ni PyTorch). Sert à :
  1. valider toute la boucle closed-loop SANS se battre contre les vieilles stacks
  2. fournir un point de comparaison honnête : tout vrai modèle SOTA branché ensuite
     devra battre cette baseline pour justifier sa complexité.

Heuristique : un piéton est jugé "intentionnel" (va traverser) s'il se rapproche
de la trajectoire de l'ego. On combine deux signaux simples et interprétables :
  - proximité latérale à la trajectoire de l'ego (plus il est proche du couloir, plus c'est risqué)
  - composante de vitesse dirigée vers la trajectoire de l'ego (il s'engage)

C'est volontairement simple : une baseline n'a pas à être bonne, elle a à être
honnête et reproductible.
"""

import math
from typing import Dict, List
from trace import AgentState
from intention.base import IntentionPredictor


class HeuristicIntention(IntentionPredictor):
    name = "heuristic_v0"

    def __init__(self, lateral_scale: float = 4.0, approach_scale: float = 2.0):
        """
        lateral_scale  : m — distance latérale au-delà de laquelle le risque décroît
        approach_scale : m/s — vitesse d'approche au-delà de laquelle l'intention sature
        """
        self.lateral_scale = lateral_scale
        self.approach_scale = approach_scale

    def predict_intent(self, ego: AgentState,
                       pedestrians: List[AgentState]) -> Dict[str, float]:
        out: Dict[str, float] = {}

        # Direction de l'ego (unitaire). Si à l'arrêt, on prend l'axe x par défaut.
        speed = ego.speed
        if speed < 0.1:
            dx, dy = 1.0, 0.0
        else:
            dx, dy = ego.vx / speed, ego.vy / speed
        # Normale (gauche) à la direction de l'ego
        nx, ny = -dy, dx

        for ped in pedestrians:
            rx, ry = ped.x - ego.x, ped.y - ego.y
            longitudinal = rx * dx + ry * dy        # devant (>0) / derrière (<0)
            lateral = rx * nx + ry * ny             # signé : côté gauche/droit
            abs_lateral = abs(lateral)

            # Piéton derrière l'ego → pas de risque de traversée devant
            if longitudinal <= 0:
                out[ped.id] = 0.0
                continue

            # Signal 1 : proximité latérale (1 si sur la trajectoire, →0 si loin)
            prox = math.exp(-abs_lateral / self.lateral_scale)

            # Signal 2 : vitesse dirigée vers la trajectoire de l'ego.
            # Projeter la vitesse du piéton sur la normale, du bon signe (vers l'ego).
            v_toward = -(ped.vx * nx + ped.vy * ny) * (1 if lateral > 0 else -1)
            approach = max(0.0, v_toward) / self.approach_scale
            approach = min(1.0, approach)

            # Combinaison : il faut être proche ET s'engager pour un score élevé.
            # On pondère la proximité par l'engagement, avec un plancher de proximité.
            intent = prox * (0.4 + 0.6 * approach)
            out[ped.id] = max(0.0, min(1.0, intent))

        return out
