#!/usr/bin/env python3
"""
rcib.scenario — Définition d'un scénario (PUR Python, testable sans CARLA).

Un ScenarioSpec décrit COMPLÈTEMENT et de façon REPRODUCTIBLE une situation de
conduite : où démarre l'ego, sa vitesse cible, où sont les piétons et comment ils
traversent. La même spec (même seed) donne toujours le même scénario.

Les coordonnées sont RELATIVES à l'ego au départ :
  - l'ego démarre à (0, 0) et roule dans la direction +x
  - x = distance longitudinale (devant l'ego = x > 0)
  - y = décalage latéral (gauche de l'ego = y > 0, par convention plan standard)
  - unités : mètres, m/s, secondes

Cette représentation relative est volontaire : le KinematicRunner l'utilise telle
quelle, et le CarlaScenarioBridge la transpose dans le repère monde de CARLA en la
plaçant à un point de spawn de la carte (rotation 2D selon le yaw du spawn point).
"""

import random
import math
from dataclasses import dataclass, field, asdict
from typing import List, Tuple, Optional


@dataclass
class PedestrianSpec:
    """
    Un piéton scripté. Il reste immobile jusqu'à `start_time`, puis marche en
    ligne droite dans `walk_direction` à `speed` (m/s).
    """
    ped_id: str
    start_x: float                       # position longitudinale de départ (m, devant l'ego)
    start_y: float                       # décalage latéral de départ (m)
    walk_direction: Tuple[float, float]  # direction de marche (dx, dy), sera normalisée
    speed: float                         # vitesse de marche (m/s)
    start_time: float = 0.0              # instant où il commence à marcher (s)

    def normalized_direction(self) -> Tuple[float, float]:
        dx, dy = self.walk_direction
        n = math.hypot(dx, dy)
        if n < 1e-9:
            return (0.0, 0.0)
        return (dx / n, dy / n)

    def position_at(self, t: float) -> Tuple[float, float]:
        """Position (x, y) du piéton à l'instant t, en cinématique pure."""
        if t <= self.start_time:
            return (self.start_x, self.start_y)
        dt = t - self.start_time
        dx, dy = self.normalized_direction()
        return (self.start_x + dx * self.speed * dt,
                self.start_y + dy * self.speed * dt)

    def velocity_at(self, t: float) -> Tuple[float, float]:
        """Vitesse (vx, vy) du piéton à l'instant t."""
        if t <= self.start_time:
            return (0.0, 0.0)
        dx, dy = self.normalized_direction()
        return (dx * self.speed, dy * self.speed)


@dataclass
class ScenarioSpec:
    """Description complète et reproductible d'un scénario."""
    scenario_id: str
    seed: int
    # Ego
    ego_target_speed: float              # vitesse de croisière visée (m/s)
    ego_goal_distance: float             # distance à parcourir pour "réussir" (m)
    # Piétons
    pedestrians: List[PedestrianSpec] = field(default_factory=list)
    # Simulation
    duration: float = 12.0               # durée max (s)
    fixed_delta: float = 0.05            # pas de temps (s) -> 20 Hz
    # Carte (utilisé par le bridge CARLA ; ignoré par le KinematicRunner)
    map_name: str = "Town10HD_Opt"

    @property
    def n_ticks(self) -> int:
        return int(self.duration / self.fixed_delta)

    @property
    def ego_goal_xy(self) -> Tuple[float, float]:
        """But de l'ego en coordonnées relatives (droit devant)."""
        return (self.ego_goal_distance, 0.0)

    def to_dict(self) -> dict:
        return asdict(self)


# ──────────────────────────────────────────────
# GÉNÉRATEURS DE SCÉNARIOS (reproductibles via seed)
# ──────────────────────────────────────────────

def crossing_scenario(seed: int = 0,
                      ego_speed: float = 8.0,
                      jitter: bool = True) -> ScenarioSpec:
    """
    Scénario canonique : UN piéton traverse perpendiculairement la trajectoire de
    l'ego, calibré pour créer un conflit (le piéton arrive sur la route au moment
    où l'ego y arrive aussi).

    Le timing est calculé pour que, SANS réaction de l'ego, il y ait un risque de
    collision — c'est exactement ce que RCIB doit mesurer.

    Avec jitter=True, une seed donnée introduit de petites variations reproductibles
    (vitesse du piéton, distance, instant de départ) → un banc de scénarios variés
    mais déterministes.
    """
    rng = random.Random(seed)

    # Paramètres de base
    cross_distance = 25.0   # le piéton apparaît à 25 m devant l'ego
    ped_speed = 1.5         # vitesse de marche typique (m/s)
    lateral_start = 6.0     # le piéton démarre à 6 m sur le côté (droite)

    if jitter:
        cross_distance += rng.uniform(-3.0, 3.0)    # 22..28 m
        ped_speed += rng.uniform(-0.2, 0.3)         # 1.3..1.8 m/s
        lateral_start += rng.uniform(-0.5, 0.5)     # 5.5..6.5 m

    # Temps pour que l'ego atteigne le point de croisement (x = cross_distance).
    # L'ego ne roule PAS à vitesse constante : il part de 0 et accélère (cruise
    # proportionnel). On modélise une phase d'accélération moyenne puis le régime.
    # Approximation : phase de montée ~ ego_speed/accel_moy, distance pendant la
    # montée ~ 0.5 * ego_speed * t_ramp ; au-delà, vitesse constante.
    accel_moy = 2.0                                  # m/s² effectif observé (cruise kp)
    t_ramp = ego_speed / accel_moy                   # temps pour atteindre la vitesse
    d_ramp = 0.5 * ego_speed * t_ramp                # distance parcourue en montant
    if cross_distance <= d_ramp:
        # Le croisement a lieu pendant la phase d'accélération
        t_ego_arrival = math.sqrt(2 * cross_distance / accel_moy)
    else:
        # Montée complète + régime à vitesse constante
        t_ego_arrival = t_ramp + (cross_distance - d_ramp) / ego_speed

    # Temps pour que le piéton traverse de lateral_start jusqu'à y=0
    t_ped_cross = lateral_start / ped_speed
    # Le piéton doit arriver sur la trajectoire ~quand l'ego y arrive
    start_time = max(0.0, t_ego_arrival - t_ped_cross)

    ped = PedestrianSpec(
        ped_id="walker_01",
        start_x=cross_distance,
        start_y=-lateral_start,             # côté droit (y négatif par convention)
        walk_direction=(0.0, 1.0),          # marche vers +y (traverse vers la gauche)
        speed=ped_speed,
        start_time=start_time,
    )

    return ScenarioSpec(
        scenario_id=f"crossing_seed{seed}",
        seed=seed,
        ego_target_speed=ego_speed,
        ego_goal_distance=cross_distance + 20.0,   # le but est au-delà du croisement
        pedestrians=[ped],
        duration=12.0,
        fixed_delta=0.05,
    )


def no_pedestrian_scenario(seed: int = 0, ego_speed: float = 8.0) -> ScenarioSpec:
    """Scénario de contrôle : route libre, aucun piéton. L'ego doit atteindre son but."""
    return ScenarioSpec(
        scenario_id=f"free_road_seed{seed}",
        seed=seed,
        ego_target_speed=ego_speed,
        ego_goal_distance=60.0,
        pedestrians=[],
        duration=12.0,
        fixed_delta=0.05,
    )
