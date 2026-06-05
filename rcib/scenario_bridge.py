#!/usr/bin/env python3
"""
rcib.scenario_bridge — Exécute un ScenarioSpec dans le VRAI CARLA (sur pod).

Produit le même objet `Trace` que le KinematicRunner, mais avec la physique réelle
de CARLA. C'est l'aboutissement de la Phase 2 : faire vivre le scénario dans le
simulateur.

⚠️ Ce module nécessite le package `carla` (donc un pod RunPod avec serveur CARLA
lancé). Il ne peut pas tourner dans un environnement sans CARLA. Toute la logique
"intelligente" (scénario, contrôle) est testée à froid via le KinematicRunner ;
ce bridge n'est qu'une coquille d'orchestration de l'API CARLA.

Points clés (issus de la Phase 0 et de la doc CARLA) :
  - mode SYNCHRONE + fixed_delta + seed -> reproductibilité
  - piétons contrôlés MANUELLEMENT (WalkerControl) -> déterministe (pas d'IA CARLA)
  - l'ego placé à un spawn point ; le repère relatif du scénario est tourné selon
    le yaw du spawn point pour placer les piétons au bon endroit dans le monde
  - capteur de collision attaché à l'ego -> détection fiable (physique CARLA)
  - nettoyage systématique des acteurs + restauration du mode async en sortie

Usage (dans le pod, serveur CARLA lancé, env chargé) :
    source ~/rcib_env.sh
    python3 -c "from scenario_bridge import run_in_carla; \
                from scenario import crossing_scenario; \
                tr = run_in_carla(crossing_scenario(seed=0)); \
                print(len(tr), 'frames')"
"""

import math
import time
from typing import Optional, Callable, List

from trace import Trace, Frame, AgentState
from scenario import ScenarioSpec, PedestrianSpec
from ego_controller import CruiseController


def _rotate(x: float, y: float, yaw_deg: float):
    """Tourne le vecteur (x, y) du repère relatif vers le repère monde selon yaw."""
    yaw = math.radians(yaw_deg)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    return (x * cos_y - y * sin_y, x * sin_y + y * cos_y)


class CarlaScenarioBridge:
    """Orchestre l'exécution d'un ScenarioSpec dans CARLA."""

    def __init__(self, host: str = "localhost", port: int = 2000,
                 timeout: float = 30.0):
        import carla  # import local : n'est requis que dans le pod
        self.carla = carla
        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout)
        self.world = None
        self.actors = []           # acteurs à nettoyer
        self.collision_sensor = None
        self._collision_flag = {"hit": False, "other": None}

    # ──────────────────────────────────────────
    def setup_world(self, spec: ScenarioSpec):
        """Charge la carte, fixe le mode synchrone + la seed."""
        carla = self.carla
        # Charger la carte demandée (ou garder la courante si déjà bonne)
        current = self.client.get_world()
        if spec.map_name not in current.get_map().name:
            self.world = self.client.load_world(spec.map_name)
        else:
            self.world = current

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = spec.fixed_delta
        self.world.apply_settings(settings)

        # Seed du trafic/piétons pour la reproductibilité
        try:
            self.world.set_pedestrians_seed(spec.seed)
        except Exception:
            pass  # certaines versions n'ont pas cette API ; non bloquant

        # Quelques ticks pour stabiliser le monde
        for _ in range(5):
            self.world.tick()

    # ──────────────────────────────────────────
    def spawn_ego(self, spec: ScenarioSpec):
        """Spawn l'ego à un point de spawn de la carte. Retourne (vehicle, transform)."""
        carla = self.carla
        bp_lib = self.world.get_blueprint_library()
        # Un modèle de véhicule simple et léger
        vehicle_bp = bp_lib.filter("vehicle.*")[0]

        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("Aucun point de spawn sur la carte")
        spawn = spawn_points[0]

        vehicle = self.world.try_spawn_actor(vehicle_bp, spawn)
        if vehicle is None:
            # Essayer d'autres points si le premier est occupé
            for sp in spawn_points[1:10]:
                vehicle = self.world.try_spawn_actor(vehicle_bp, sp)
                if vehicle:
                    spawn = sp
                    break
        if vehicle is None:
            raise RuntimeError("Impossible de spawn l'ego")

        self.actors.append(vehicle)
        return vehicle, spawn

    def attach_collision_sensor(self, vehicle):
        """Attache un capteur de collision à l'ego (détection fiable)."""
        carla = self.carla
        bp = self.world.get_blueprint_library().find("sensor.other.collision")
        sensor = self.world.spawn_actor(bp, carla.Transform(), attach_to=vehicle)

        def _on_collision(event):
            self._collision_flag["hit"] = True
            other = getattr(event, "other_actor", None)
            if other is not None:
                self._collision_flag["other"] = other.type_id

        sensor.listen(_on_collision)
        self.collision_sensor = sensor
        self.actors.append(sensor)

    def spawn_pedestrians(self, spec: ScenarioSpec, ego_transform) -> List:
        """
        Spawn les piétons en transposant leurs coordonnées relatives dans le monde,
        selon la position et le yaw de l'ego.
        Retourne la liste des acteurs walkers (dans l'ordre de spec.pedestrians).
        """
        carla = self.carla
        bp_lib = self.world.get_blueprint_library()
        walker_bps = bp_lib.filter("walker.pedestrian.*")

        ego_loc = ego_transform.location
        ego_yaw = ego_transform.rotation.yaw
        walkers = []

        for i, ps in enumerate(spec.pedestrians):
            # Position monde = position ego + rotation(yaw) * offset relatif
            wx, wy = _rotate(ps.start_x, ps.start_y, ego_yaw)
            spawn_loc = carla.Location(x=ego_loc.x + wx, y=ego_loc.y + wy,
                                       z=ego_loc.z + 1.0)  # +1m pour éviter le sol
            spawn_tf = carla.Transform(spawn_loc)
            walker_bp = walker_bps[i % len(walker_bps)]
            walker = self.world.try_spawn_actor(walker_bp, spawn_tf)
            walkers.append(walker)  # peut être None si échec (on gère plus bas)
            if walker:
                self.actors.append(walker)

        # Quelques ticks pour stabiliser les spawns
        for _ in range(2):
            self.world.tick()
        return walkers

    # ──────────────────────────────────────────
    def run(self, spec: ScenarioSpec,
            controller: Optional[CruiseController] = None,
            planner: Optional[Callable] = None) -> Trace:
        """
        Exécute le scénario complet et retourne une Trace.
        `planner` (Phase 3) : fonction(ego_state, peds, t) -> vitesse cible.
        """
        carla = self.carla
        if controller is None:
            controller = CruiseController(target_speed=spec.ego_target_speed)

        self.setup_world(spec)
        vehicle, ego_tf = self.spawn_ego(spec)
        self.attach_collision_sensor(vehicle)
        walkers = self.spawn_pedestrians(spec, ego_tf)

        # Yaw de l'ego (pour transposer les directions de marche des piétons)
        ego_yaw = ego_tf.rotation.yaw

        frames = []
        dt = spec.fixed_delta

        try:
            for i in range(spec.n_ticks):
                t = i * dt

                # ── Contrôler les piétons (WalkerControl manuel) ──
                ped_states = []
                for ps, walker in zip(spec.pedestrians, walkers):
                    if walker is None:
                        continue
                    # Direction de marche transposée dans le repère monde
                    ddx, ddy = ps.normalized_direction()
                    wdx, wdy = _rotate(ddx, ddy, ego_yaw)
                    speed = ps.speed if t >= ps.start_time else 0.0
                    control = carla.WalkerControl()
                    control.direction = carla.Vector3D(x=wdx, y=wdy, z=0.0)
                    control.speed = speed
                    walker.apply_control(control)

                    loc = walker.get_location()
                    vel = walker.get_velocity()
                    ped_states.append(AgentState(
                        id=ps.ped_id, x=loc.x, y=loc.y, vx=vel.x, vy=vel.y))

                # ── État de l'ego ──
                loc = vehicle.get_location()
                vel = vehicle.get_velocity()
                ego_speed = math.hypot(vel.x, vel.y)
                ego_state = AgentState(id="ego", x=loc.x, y=loc.y, vx=vel.x, vy=vel.y)

                # ── (Phase 3) planner module la vitesse cible ──
                if planner is not None:
                    controller.set_target_speed(planner(ego_state, ped_states, t))

                # ── Contrôler l'ego ──
                cmd = controller.control(ego_speed)
                vehicle.apply_control(carla.VehicleControl(
                    throttle=cmd.throttle, brake=cmd.brake, steer=cmd.steer))

                # ── Enregistrer la frame ──
                frames.append(Frame(
                    t=t, ego=ego_state, pedestrians=ped_states,
                    throttle=cmd.throttle, brake=cmd.brake,
                    collision=self._collision_flag["hit"],
                    collision_with=self._collision_flag["other"],
                ))

                self.world.tick()
        finally:
            self.cleanup()

        # But en coordonnées monde (pour reached_goal)
        gx, gy = _rotate(spec.ego_goal_distance, 0.0, ego_yaw)
        goal_world = (ego_tf.location.x + gx, ego_tf.location.y + gy)

        return Trace(
            frames=frames,
            scenario_id=spec.scenario_id,
            seed=spec.seed,
            ego_goal=goal_world,
            goal_radius=5.0,
            map_name=spec.map_name,
            carla_version=self.client.get_server_version(),
            intention_model="none",
        )

    # ──────────────────────────────────────────
    def cleanup(self):
        """Détruit les acteurs et restaure le mode asynchrone."""
        if self.collision_sensor is not None:
            try:
                self.collision_sensor.stop()
            except Exception:
                pass
        for actor in self.actors:
            try:
                actor.destroy()
            except Exception:
                pass
        self.actors = []
        if self.world is not None:
            try:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                self.world.apply_settings(settings)
            except Exception:
                pass


def run_in_carla(spec: ScenarioSpec,
                 controller: Optional[CruiseController] = None,
                 planner: Optional[Callable] = None,
                 host: str = "localhost", port: int = 2000) -> Trace:
    """Raccourci : exécute un scénario dans CARLA et retourne la Trace."""
    bridge = CarlaScenarioBridge(host=host, port=port)
    return bridge.run(spec, controller=controller, planner=planner)
