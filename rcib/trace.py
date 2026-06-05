#!/usr/bin/env python3
"""
rcib.trace — Modèle de données d'une trace de simulation.

Une "trace" est la séquence d'états au cours du temps produite par une run closed-loop.
C'est le CONTRAT central du système : le Scenario Bridge (Phase 2) la produit, le
Metrics Harness (Phase 1) la consomme. En la définissant proprement, le harness devient
testable SANS CARLA — on lui donne des traces synthétiques.

Conventions d'unités (cohérentes partout) :
  - positions   : mètres (x, y) dans le plan
  - vitesses    : m/s (vx, vy)
  - temps       : secondes (t)
  - angles      : non utilisés ici (on travaille en vecteurs)
"""

from dataclasses import dataclass, field, asdict
from typing import List, Optional
import math


@dataclass
class AgentState:
    """État d'un agent (ego ou piéton) à un instant donné."""
    id: str
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0

    @property
    def speed(self) -> float:
        """Norme de la vitesse (m/s)."""
        return math.hypot(self.vx, self.vy)

    def distance_to(self, other: "AgentState") -> float:
        """Distance euclidienne à un autre agent (m)."""
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass
class Frame:
    """Un instantané de la simulation à un tick donné."""
    t: float                                  # temps (s)
    ego: AgentState                           # l'ego véhicule
    pedestrians: List[AgentState] = field(default_factory=list)
    throttle: float = 0.0                     # commande appliquée [0,1]
    brake: float = 0.0                        # commande appliquée [0,1]
    collision: bool = False                   # event de collision CARLA (si dispo)
    collision_with: Optional[str] = None      # id du piéton percuté (si connu)


@dataclass
class Trace:
    """Une run complète : métadonnées + séquence de frames."""
    frames: List[Frame] = field(default_factory=list)
    # Métadonnées de reproductibilité
    scenario_id: str = "unknown"
    seed: Optional[int] = None
    ego_goal: Optional[tuple] = None          # (x, y) destination visée
    goal_radius: float = 3.0                   # m : rayon pour considérer le but atteint
    map_name: str = "unknown"
    carla_version: str = "unknown"
    intention_model: str = "unknown"

    def __len__(self) -> int:
        return len(self.frames)

    def duration(self) -> float:
        """Durée totale de la trace (s)."""
        if len(self.frames) < 2:
            return 0.0
        return self.frames[-1].t - self.frames[0].t

    def reached_goal(self) -> bool:
        """
        L'ego a-t-il atteint sa destination ?
        Vrai s'il est passé à moins de `goal_radius` du but à N'IMPORTE QUEL instant
        (pas seulement à la fin) : une fois le but franchi, il reste atteint même si
        l'ego continue de rouler au-delà.
        """
        if self.ego_goal is None or not self.frames:
            return False
        gx, gy = self.ego_goal
        for f in self.frames:
            if math.hypot(f.ego.x - gx, f.ego.y - gy) <= self.goal_radius:
                return True
        return False

    def to_dict(self) -> dict:
        """Sérialisation JSON-compatible (pour le Run Logger)."""
        return {
            "scenario_id": self.scenario_id,
            "seed": self.seed,
            "ego_goal": list(self.ego_goal) if self.ego_goal else None,
            "goal_radius": self.goal_radius,
            "map_name": self.map_name,
            "carla_version": self.carla_version,
            "intention_model": self.intention_model,
            "frames": [asdict(f) for f in self.frames],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Trace":
        """Reconstruit une Trace depuis un dict (pour le replay)."""
        frames = []
        for fd in d.get("frames", []):
            ego = AgentState(**fd["ego"])
            peds = [AgentState(**p) for p in fd.get("pedestrians", [])]
            frames.append(Frame(
                t=fd["t"], ego=ego, pedestrians=peds,
                throttle=fd.get("throttle", 0.0), brake=fd.get("brake", 0.0),
                collision=fd.get("collision", False),
                collision_with=fd.get("collision_with"),
            ))
        goal = d.get("ego_goal")
        return cls(
            frames=frames,
            scenario_id=d.get("scenario_id", "unknown"),
            seed=d.get("seed"),
            ego_goal=tuple(goal) if goal else None,
            goal_radius=d.get("goal_radius", 3.0),
            map_name=d.get("map_name", "unknown"),
            carla_version=d.get("carla_version", "unknown"),
            intention_model=d.get("intention_model", "unknown"),
        )
