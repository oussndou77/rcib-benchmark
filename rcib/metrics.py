#!/usr/bin/env python3
"""
rcib.metrics — Metrics Harness.

Transforme une trace de simulation en scores de CONDUITE (pas de prédiction).
C'est la contribution différenciante de RCIB : là où le SOTA mesure ADE/FDE
(qualité de prédiction open-loop), on mesure ce qui compte pour le déploiement —
collisions, marge de sécurité, confort — sur une boucle fermée.

Métriques produites :
  - collision        : la run a-t-elle produit un contact ? + gravité (vitesse à l'impact)
  - min_ttc          : marge de sécurité minimale (time-to-collision, s) sur toute la run
  - max_decel        : décélération maximale (m/s²) — proxy de freinage d'urgence
  - rms_jerk         : à-coups (m/s³) — confort longitudinal
  - n_hard_brakes    : nombre d'événements de freinage brusque
  - reached_goal     : l'ego a-t-il atteint sa destination ?
  - rcib_score       : score composite [0,1] unifiant sécurité + confort + réussite

NOTE sur les seuils : les valeurs par défaut (TTC sûr = 3 s, freinage brusque à
3.9 m/s² ≈ 0.4 g, etc.) sont des conventions courantes en sécurité routière, mais
elles sont TOUTES configurables via MetricsConfig. Le harness ne prétend pas qu'il
existe un seuil universel — il rend les choix explicites et ajustables.
"""

from dataclasses import dataclass, asdict
from typing import List, Optional, Dict
import math

from trace import Trace, Frame, AgentState


# ──────────────────────────────────────────────
# CONFIG (seuils explicites, pas de nombres magiques cachés)
# ──────────────────────────────────────────────

@dataclass
class MetricsConfig:
    # Géométrie de collision
    collision_radius: float = 1.5      # m : somme des rayons ego+piéton pour un contact
    ego_corridor_halfwidth: float = 1.5  # m : demi-largeur du couloir de l'ego pour le TTC

    # Sécurité
    ttc_safe: float = 3.0              # s : au-delà, la marge est jugée confortable
    ttc_cap: float = 10.0              # s : valeur de plafonnement (pas de conflit → cap)

    # Confort
    hard_brake_decel: float = 3.9      # m/s² (~0.4 g) : seuil de freinage brusque
    decel_max_tolerable: float = 8.0   # m/s² : décélération quasi-urgence (confort = 0)

    # Poids du score composite RCIB (somment à 1)
    w_safety: float = 0.5
    w_comfort: float = 0.2
    w_success: float = 0.3


# ──────────────────────────────────────────────
# PHYSIQUE : TTC point-masse avec rayon de collision
# ──────────────────────────────────────────────

def instantaneous_ttc(ego: AgentState, ped: AgentState, collision_radius: float,
                      cap: float) -> float:
    """
    Time-to-collision instantané entre ego et piéton, en supposant des vitesses
    constantes à partir de cet instant (modèle point-masse, rayon R).

    On cherche le plus petit t >= 0 tel que |r + v·t| = R, où
      r = position relative (ped - ego), v = vitesse relative (ped - ego).
    Cela donne une équation quadratique : |v|² t² + 2(r·v) t + (|r|² - R²) = 0.

    Retourne le TTC (s), ou `cap` s'il n'y a pas de collision prédite (agents
    qui s'éloignent ou se manquent).
    """
    rx, ry = ped.x - ego.x, ped.y - ego.y
    vx, vy = ped.vx - ego.vx, ped.vy - ego.vy

    rr = rx * rx + ry * ry          # |r|²
    R2 = collision_radius * collision_radius

    # Déjà en contact à cet instant
    if rr <= R2:
        return 0.0

    a = vx * vx + vy * vy            # |v|²
    if a < 1e-9:
        return cap                   # vitesse relative nulle → jamais de contact

    b = 2.0 * (rx * vx + ry * vy)    # 2 (r·v)
    c = rr - R2

    # S'ils s'éloignent (r·v >= 0) et pas déjà en contact → pas de collision
    if b >= 0:
        return cap

    disc = b * b - 4.0 * a * c
    if disc < 0:
        return cap                   # ils se manquent

    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    if t1 >= 0:
        return min(t1, cap)
    t2 = (-b + sqrt_disc) / (2.0 * a)
    if t2 >= 0:
        return min(t2, cap)
    return cap


def _in_ego_corridor(ego: AgentState, ped: AgentState, halfwidth: float) -> bool:
    """
    Le piéton est-il dans le couloir longitudinal de l'ego (devant lui) ?
    On projette la position relative sur la direction de l'ego ; le piéton compte
    s'il est devant (projection longitudinale > 0) et latéralement proche.
    Si l'ego est ~immobile, on ne peut pas définir de direction → on prend tout.
    """
    speed = ego.speed
    if speed < 0.1:
        return True  # ego quasi à l'arrêt : pas de direction fiable, on n'exclut rien
    # Vecteur direction de l'ego (unitaire)
    dx, dy = ego.vx / speed, ego.vy / speed
    rx, ry = ped.x - ego.x, ped.y - ego.y
    longitudinal = rx * dx + ry * dy           # projection avant/arrière
    lateral = abs(rx * (-dy) + ry * dx)        # distance latérale
    return longitudinal > 0 and lateral <= halfwidth


# ──────────────────────────────────────────────
# DÉRIVÉES NUMÉRIQUES : accélération, jerk
# ──────────────────────────────────────────────

def _longitudinal_accel_series(trace: Trace) -> List[float]:
    """
    Série de l'accélération longitudinale de l'ego (m/s²), dérivée de la vitesse.
    Signe : négatif = décélération. Calculée par différences finies sur la norme
    de vitesse (suffisant pour le confort longitudinal).
    """
    frames = trace.frames
    accels = []
    for i in range(1, len(frames)):
        dt = frames[i].t - frames[i - 1].t
        if dt <= 1e-6:
            accels.append(0.0)
            continue
        dv = frames[i].ego.speed - frames[i - 1].ego.speed
        accels.append(dv / dt)
    return accels


def _jerk_series(accels: List[float], dts: List[float]) -> List[float]:
    """Série du jerk (m/s³), dérivée de l'accélération."""
    jerks = []
    for i in range(1, len(accels)):
        dt = dts[i] if i < len(dts) else 0.0
        if dt <= 1e-6:
            jerks.append(0.0)
            continue
        jerks.append((accels[i] - accels[i - 1]) / dt)
    return jerks


# ──────────────────────────────────────────────
# RÉSULTAT
# ──────────────────────────────────────────────

@dataclass
class MetricsResult:
    collision: bool
    collision_speed: float        # m/s à l'impact (gravité), 0 si pas de collision
    min_ttc: float                # s
    max_decel: float              # m/s² (valeur positive = intensité de décélération)
    rms_jerk: float               # m/s³
    n_hard_brakes: int
    reached_goal: bool
    rcib_score: float             # [0,1]
    # sous-scores (pour la transparence / le debug)
    safety_score: float
    comfort_score: float
    success_score: float

    def to_dict(self) -> dict:
        return asdict(self)

    def summary(self) -> str:
        coll = f"COLLISION@{self.collision_speed:.1f}m/s" if self.collision else "no-collision"
        return (f"RCIB={self.rcib_score:.3f} | {coll} | "
                f"min_TTC={self.min_ttc:.2f}s | max_decel={self.max_decel:.2f}m/s² | "
                f"rms_jerk={self.rms_jerk:.2f} | hard_brakes={self.n_hard_brakes} | "
                f"goal={'✓' if self.reached_goal else '✗'}")


# ──────────────────────────────────────────────
# LE HARNESS
# ──────────────────────────────────────────────

def evaluate(trace: Trace, config: Optional[MetricsConfig] = None) -> MetricsResult:
    """
    Évalue une trace et retourne les métriques de conduite + le score RCIB.
    Fonctionne entièrement hors-ligne (aucune dépendance CARLA/GPU).
    """
    cfg = config or MetricsConfig()
    frames = trace.frames

    if len(frames) < 2:
        # Trace dégénérée
        return MetricsResult(
            collision=False, collision_speed=0.0, min_ttc=cfg.ttc_cap,
            max_decel=0.0, rms_jerk=0.0, n_hard_brakes=0,
            reached_goal=trace.reached_goal(),
            rcib_score=0.0, safety_score=0.0, comfort_score=0.0, success_score=0.0,
        )

    # ── 1. Collision ──
    collision = False
    collision_speed = 0.0
    for f in frames:
        # (a) event explicite CARLA
        if f.collision:
            collision = True
            collision_speed = max(collision_speed, f.ego.speed)
        # (b) détection géométrique (modèle de secours / hors CARLA)
        for ped in f.pedestrians:
            if f.ego.distance_to(ped) <= cfg.collision_radius:
                collision = True
                collision_speed = max(collision_speed, f.ego.speed)
    # ── 2. min TTC (uniquement sur les piétons dans le couloir de l'ego) ──
    min_ttc = cfg.ttc_cap
    for f in frames:
        for ped in f.pedestrians:
            if _in_ego_corridor(f.ego, ped, cfg.ego_corridor_halfwidth):
                ttc = instantaneous_ttc(f.ego, ped, cfg.collision_radius, cfg.ttc_cap)
                min_ttc = min(min_ttc, ttc)

    # ── 3. Confort : décélération max, jerk RMS, freinages brusques ──
    dts = [frames[i].t - frames[i - 1].t for i in range(1, len(frames))]
    accels = _longitudinal_accel_series(trace)
    jerks = _jerk_series(accels, dts)

    decels = [-a for a in accels if a < 0]        # intensités de décélération
    max_decel = max(decels) if decels else 0.0
    n_hard_brakes = sum(1 for d in decels if d >= cfg.hard_brake_decel)
    rms_jerk = math.sqrt(sum(j * j for j in jerks) / len(jerks)) if jerks else 0.0

    # ── 4. Réussite ──
    reached_goal = trace.reached_goal()

    # ── 5. Sous-scores [0,1] ──
    # Sécurité : 0 si collision ; sinon proportionnelle à la marge TTC.
    if collision:
        safety_score = 0.0
    else:
        safety_score = max(0.0, min(1.0, min_ttc / cfg.ttc_safe))

    # Confort : 1 si pas de freinage fort ; décroît jusqu'à 0 à la décél. quasi-urgence.
    if max_decel <= cfg.hard_brake_decel:
        comfort_score = 1.0
    else:
        span = max(1e-6, cfg.decel_max_tolerable - cfg.hard_brake_decel)
        comfort_score = max(0.0, min(1.0, 1.0 - (max_decel - cfg.hard_brake_decel) / span))

    # Réussite : 1 si but atteint ET pas de collision.
    success_score = 1.0 if (reached_goal and not collision) else 0.0

    # ── 6. Score composite RCIB ──
    rcib_score = (cfg.w_safety * safety_score
                  + cfg.w_comfort * comfort_score
                  + cfg.w_success * success_score)

    return MetricsResult(
        collision=collision, collision_speed=collision_speed,
        min_ttc=min_ttc, max_decel=max_decel, rms_jerk=rms_jerk,
        n_hard_brakes=n_hard_brakes, reached_goal=reached_goal,
        rcib_score=rcib_score, safety_score=safety_score,
        comfort_score=comfort_score, success_score=success_score,
    )
