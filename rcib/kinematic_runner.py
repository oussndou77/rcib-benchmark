#!/usr/bin/env python3
"""
rcib.kinematic_runner — Exécute un ScenarioSpec en cinématique pure (sans CARLA).

Produit exactement le même objet `Trace` que le bridge CARLA, mais avec une
physique simplifiée :
  - les piétons suivent leur trajectoire scriptée (position analytique)
  - l'ego avance selon une dynamique longitudinale simple :
        accélération = f(throttle, brake)   puis   v += a*dt,  x += v*dt
  - collision détectée géométriquement (le harness gère aussi ce champ)

But : valider TOUTE la chaîne (scénario → contrôle → trace → métriques) à froid,
et fournir une référence de comportement. Le bridge CARLA remplacera cette physique
simplifiée par la vraie simulation, sans changer le reste.

NB : ce runner est aussi le banc d'essai de la logique réactive de la Phase 3 —
on pourra y brancher un IntentionPredictor + un planner et tout tester sans GPU.
"""

import math
from typing import Optional, Callable

from trace import Trace, Frame, AgentState
from scenario import ScenarioSpec
from ego_controller import CruiseController, ControlCommand


# Dynamique longitudinale simplifiée de l'ego
MAX_ACCEL = 3.0      # m/s² à pleine accélération (throttle=1)
MAX_DECEL = 8.0      # m/s² à plein freinage (brake=1)


def _ego_accel(cmd: ControlCommand) -> float:
    """Accélération longitudinale (m/s²) résultant d'une commande."""
    return cmd.throttle * MAX_ACCEL - cmd.brake * MAX_DECEL


def run_kinematic(spec: ScenarioSpec,
                  controller: Optional[CruiseController] = None,
                  planner: Optional[Callable] = None,
                  collision_radius: float = 1.5) -> Trace:
    """
    Simule le scénario en cinématique pure et retourne une Trace.

    Args:
        spec        : la définition du scénario
        controller  : le cruise control (par défaut : vitesse cible constante du spec)
        planner     : optionnel (Phase 3) — fonction(ego, peds, t) -> vitesse cible.
                      Si fourni, il module la cible du controller à chaque tick.
        collision_radius : seuil de contact géométrique (m)

    L'ego avance en ligne droite selon +x (repère relatif du scénario).
    """
    if controller is None:
        controller = CruiseController(target_speed=spec.ego_target_speed)

    dt = spec.fixed_delta
    frames = []

    # État initial de l'ego : à l'origine, immobile, orienté +x
    ego_x, ego_y = 0.0, 0.0
    ego_v = 0.0    # vitesse scalaire (m/s), dirigée selon +x

    # État des piétons (incrémental, pour gérer le déclenchement par position) :
    # position courante, et drapeau "en train de marcher".
    ped_state = []
    for ps in spec.pedestrians:
        ped_state.append({"x": ps.start_x, "y": ps.start_y, "walking": False, "spec": ps})

    collided = False
    collided_with = None

    for i in range(spec.n_ticks):
        t = i * dt

        # Mettre à jour piétons : déclenchement (temps ou position) puis déplacement
        peds = []
        for st in ped_state:
            ps = st["spec"]
            if not st["walking"] and ps.should_start(t, ego_longitudinal=ego_x):
                st["walking"] = True
            if st["walking"]:
                ddx, ddy = ps.normalized_direction()
                st["x"] += ddx * ps.speed * dt
                st["y"] += ddy * ps.speed * dt
                vx, vy = ddx * ps.speed, ddy * ps.speed
            else:
                vx, vy = 0.0, 0.0
            peds.append(AgentState(id=ps.ped_id, x=st["x"], y=st["y"], vx=vx, vy=vy))

        # État courant de l'ego
        ego_state = AgentState(id="ego", x=ego_x, y=ego_y, vx=ego_v, vy=0.0)

        # (Phase 3) un planner peut moduler la vitesse cible selon l'intention
        if planner is not None:
            target = planner(ego_state, peds, t)
            controller.set_target_speed(target)

        # Commande de l'ego + dynamique longitudinale
        cmd = controller.control(ego_v)
        a = _ego_accel(cmd)
        ego_v = max(0.0, ego_v + a * dt)     # pas de marche arrière

        # Détection de collision géométrique
        for p in peds:
            if math.hypot(ego_x - p.x, ego_y - p.y) <= collision_radius:
                collided = True
                collided_with = p.id

        # Enregistrer la frame AVANT de déplacer l'ego (état à l'instant t)
        frames.append(Frame(
            t=t, ego=ego_state, pedestrians=peds,
            throttle=cmd.throttle, brake=cmd.brake,
            collision=collided, collision_with=collided_with,
        ))

        # Avancer l'ego
        ego_x += ego_v * dt

        # Terminaison anticipée : dès que l'ego atteint son but, on arrête (l'épisode
        # est fini). Évite de simuler l'ego au-delà du but — important dans CARLA où
        # rouler trop loin sort du couloir dégagé et percute un obstacle.
        if ego_x >= spec.ego_goal_distance:
            break

    return Trace(
        frames=frames,
        scenario_id=spec.scenario_id,
        seed=spec.seed,
        ego_goal=spec.ego_goal_xy,
        goal_radius=5.0,
        map_name="kinematic",
        carla_version="none",
        intention_model="none",
    )
