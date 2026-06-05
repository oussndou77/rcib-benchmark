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
    trigger_distance: Optional[float] = None  # si défini : démarre quand l'ego est à
                                              # cette distance longitudinale du croisement
                                              # (plus robuste que start_time, indép. de
                                              #  l'accélération de l'ego — cf. validation CARLA)

    def normalized_direction(self) -> Tuple[float, float]:
        dx, dy = self.walk_direction
        n = math.hypot(dx, dy)
        if n < 1e-9:
            return (0.0, 0.0)
        return (dx / n, dy / n)

    def should_start(self, t: float, ego_longitudinal: float) -> bool:
        """
        Le piéton doit-il commencer à marcher à cet instant ?
        - mode position (trigger_distance défini) : quand l'ego s'est approché à
          moins de trigger_distance du point de croisement (= start_x en longitudinal),
          sans l'avoir dépassé.
        - mode temps (sinon) : quand t >= start_time.
        `ego_longitudinal` est la position longitudinale de l'ego dans le repère du
        scénario (l'axe selon lequel l'ego avance).
        """
        if self.trigger_distance is not None:
            gap = self.start_x - ego_longitudinal     # distance ego -> point de croisement
            return 0.0 < gap <= self.trigger_distance
        return t >= self.start_time

    def position_at(self, t: float) -> Tuple[float, float]:
        """Position (x, y) du piéton à l'instant t, en cinématique pure (mode temps)."""
        if t <= self.start_time:
            return (self.start_x, self.start_y)
        dt = t - self.start_time
        dx, dy = self.normalized_direction()
        return (self.start_x + dx * self.speed * dt,
                self.start_y + dy * self.speed * dt)

    def velocity_at(self, t: float) -> Tuple[float, float]:
        """Vitesse (vx, vy) du piéton à l'instant t (mode temps)."""
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
    l'ego, déclenché par la POSITION de l'ego (pas par un temps fixe).

    Pourquoi déclenchement par position : le timing par temps fixe dépend de la
    courbe d'accélération de l'ego, qui diffère entre le KinematicRunner et CARLA
    (validation CARLA : CARLA accélère ~2× plus vite). Avec un déclenchement par
    distance, le conflit ne dépend QUE de la vitesse de croisière et de la géométrie
    — il est donc reproductible dans les deux exécuteurs. C'est aussi la façon dont
    les frameworks de scénarios CARLA (Scenario Runner) construisent leurs conflits.

    Calibrage du conflit : quand l'ego arrive à `trigger_distance` du point de
    croisement (à vitesse de croisière v), le piéton part. Il doit parcourir L
    latéralement pendant que l'ego parcourt trigger_distance :
        L = v_ped * trigger_distance / v
    On choisit L légèrement inférieur pour que le piéton soit en train de traverser
    la voie quand l'ego arrive → collision si l'ego ne ralentit pas (passif), et
    évitement s'il ralentit (réactif, il arrive après que le piéton a dégagé).
    """
    rng = random.Random(seed)

    # Le piéton apparaît loin devant (immobile), à un point de croisement fixe
    crossing_x = 40.0          # point de croisement longitudinal (m devant l'ego au départ)
    trigger_distance = 15.0    # l'ego déclenche la traversée à 15 m du croisement
    ped_speed = 1.5            # vitesse de marche (m/s)

    if jitter:
        crossing_x += rng.uniform(-4.0, 4.0)        # 36..44 m
        trigger_distance += rng.uniform(-2.0, 2.0)  # 13..17 m
        ped_speed += rng.uniform(-0.2, 0.3)         # 1.3..1.8 m/s

    # Distance latérale calibrée pour que le piéton soit dans la voie à l'arrivée.
    # Facteur 0.85 : le piéton atteint y=0 un peu avant l'ego -> collision franche
    # pour le passif (le piéton est en plein milieu de la voie au passage).
    lateral_start = 0.85 * ped_speed * trigger_distance / ego_speed

    ped = PedestrianSpec(
        ped_id="walker_01",
        start_x=crossing_x,
        start_y=-lateral_start,             # côté droit (y négatif)
        walk_direction=(0.0, 1.0),          # traverse vers +y
        speed=ped_speed,
        trigger_distance=trigger_distance,  # déclenchement par position
    )

    return ScenarioSpec(
        scenario_id=f"crossing_seed{seed}",
        seed=seed,
        ego_target_speed=ego_speed,
        ego_goal_distance=crossing_x + 20.0,   # le but est au-delà du croisement
        pedestrians=[ped],
        duration=14.0,
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
