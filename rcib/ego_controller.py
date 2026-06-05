#!/usr/bin/env python3
"""
rcib.ego_controller — Contrôle de la vitesse de l'ego (PUR Python, testable).

Un cruise control proportionnel : il maintient une vitesse cible en dosant
accélérateur/frein. C'est volontairement simple et déterministe.

En Phase 2, la vitesse cible est CONSTANTE (l'ego ne réagit pas aux piétons → c'est
le cas "passif" qui sert de baseline). En Phase 3, l'Ego Planner réactif modulera
dynamiquement `target_speed` selon l'intention prédite (ralentir/freiner). Le
contrôleur, lui, ne change pas — il traduit juste "vitesse voulue" en commandes.
"""

from dataclasses import dataclass


@dataclass
class ControlCommand:
    """Commande à appliquer au véhicule (compatibles avec carla.VehicleControl)."""
    throttle: float   # [0, 1]
    brake: float      # [0, 1]
    steer: float = 0.0


class CruiseController:
    """Maintient une vitesse cible via un contrôleur proportionnel simple."""

    def __init__(self, target_speed: float, kp: float = 0.5,
                 brake_kp: float = 0.5, max_throttle: float = 0.75):
        """
        target_speed : vitesse visée (m/s)
        kp           : gain proportionnel pour l'accélérateur
        brake_kp     : gain proportionnel pour le frein
        max_throttle : plafond d'accélérateur (évite les accélérations brutales)
        """
        self.target_speed = target_speed
        self.kp = kp
        self.brake_kp = brake_kp
        self.max_throttle = max_throttle

    def set_target_speed(self, speed: float) -> None:
        """Permet à un planner (Phase 3) de moduler la cible dynamiquement."""
        self.target_speed = max(0.0, speed)

    def control(self, current_speed: float) -> ControlCommand:
        """
        Calcule la commande pour rapprocher current_speed de target_speed.
        - si on est trop lent  -> accélérer (throttle proportionnel à l'écart)
        - si on est trop rapide -> freiner  (brake proportionnel à l'écart)
        """
        error = self.target_speed - current_speed

        if self.target_speed <= 0.01:
            # Cible = arrêt complet : freiner franchement
            return ControlCommand(throttle=0.0, brake=1.0)

        if error > 0.1:
            throttle = min(self.max_throttle, self.kp * error)
            return ControlCommand(throttle=throttle, brake=0.0)
        elif error < -0.1:
            brake = min(1.0, self.brake_kp * (-error))
            return ControlCommand(throttle=0.0, brake=brake)
        else:
            # Dans la zone morte : laisser rouler (ni gaz ni frein)
            return ControlCommand(throttle=0.0, brake=0.0)
