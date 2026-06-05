#!/usr/bin/env python3
"""
rcib.intention.velocity — Baseline d'intention basée sur la VITESSE (V0 alternative).

Signal différent de la baseline heuristique : ici on raisonne en "où sera le piéton",
pas "où il est". On extrapole sa trajectoire en ligne droite et on regarde s'il va
COUPER la trajectoire de l'ego dans un horizon court. C'est une approche "constant
velocity" — la même hypothèse que le baseline classique en prédiction de trajectoire.

Pourquoi une 2e baseline ? Pour que RCIB ait des compétiteurs aux philosophies
distinctes : la heuristique réagit à la position actuelle, celle-ci anticipe via la
vitesse. Comparer les deux montre que le benchmark capture la différence — c'est tout
l'intérêt d'un banc d'essai.
"""

import math
from typing import Dict, List
from trace import AgentState
from intention.base import IntentionPredictor


class ConstantVelocityIntention(IntentionPredictor):
    name = "constant_velocity_v0"

    def __init__(self, horizon: float = 3.0, corridor_halfwidth: float = 2.5):
        """
        horizon            : s — sur combien de temps on extrapole la trajectoire
        corridor_halfwidth : m — demi-largeur du couloir de l'ego (zone de conflit)
        """
        self.horizon = horizon
        self.corridor_halfwidth = corridor_halfwidth

    def predict_intent(self, ego: AgentState,
                       pedestrians: List[AgentState]) -> Dict[str, float]:
        out: Dict[str, float] = {}

        # Direction de l'ego (unitaire)
        speed = ego.speed
        if speed < 0.1:
            dx, dy = 1.0, 0.0
        else:
            dx, dy = ego.vx / speed, ego.vy / speed
        nx, ny = -dy, dx   # normale (latéral)

        for ped in pedestrians:
            rx, ry = ped.x - ego.x, ped.y - ego.y
            longitudinal = rx * dx + ry * dy
            if longitudinal <= 0:           # derrière l'ego
                out[ped.id] = 0.0
                continue

            # Écart latéral actuel du piéton à la trajectoire de l'ego
            lateral_now = rx * nx + ry * ny

            # Composante latérale de la vitesse du piéton (vers/loin de la trajectoire)
            v_lat = ped.vx * nx + ped.vy * ny

            # Si le piéton ne bouge pas latéralement, son intention dépend juste
            # de s'il est déjà dans le couloir.
            if abs(v_lat) < 1e-3:
                inside = abs(lateral_now) <= self.corridor_halfwidth
                out[ped.id] = 0.3 if inside else 0.0
                continue

            # Temps pour qu'il atteigne la trajectoire de l'ego (lateral -> 0)
            t_cross = -lateral_now / v_lat   # >0 s'il se dirige vers la trajectoire

            if t_cross < 0 or t_cross > self.horizon:
                # il s'éloigne, ou croisera trop tard pour être une menace
                # (un petit score résiduel s'il est déjà dans le couloir)
                inside = abs(lateral_now) <= self.corridor_halfwidth
                out[ped.id] = 0.2 if inside else 0.0
                continue

            # Il va couper la trajectoire dans l'horizon : intention élevée,
            # d'autant plus que le croisement est proche dans le temps.
            intent = 1.0 - (t_cross / self.horizon) * 0.5   # 0.5..1.0
            out[ped.id] = max(0.0, min(1.0, intent))

        return out
