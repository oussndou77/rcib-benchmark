#!/usr/bin/env python3
"""
rcib.benchmark — Batch runner + leaderboard (PHASE 5, aboutissement de RCIB).

Fait tourner plusieurs prédicteurs d'intention sur une GRILLE de scénarios à seeds
fixées, agrège les métriques de conduite PAR MODÈLE, et produit un leaderboard
reproductible. C'est l'outil qui transforme RCIB en un vrai banc d'essai utilisable.

Différence avec run_logger.build_leaderboard (qui agrège run par run) : ici on agrège
PAR MODÈLE sur tous les scénarios (un modèle = une moyenne sur M seeds), ce qui est la
vue qu'on veut pour comparer des prédicteurs.

Reproductibilité : la grille de scénarios est déterminée par des seeds fixées ; la même
suite (`BenchmarkSuite`) donne toujours le même classement. Les résultats sont
sérialisables en JSON et commitables (vérifiabilité — ce que le SOTA ne fait pas).

Agnostique à l'exécuteur : par défaut le KinematicRunner (sans GPU, pour le dev et la
CI), mais on peut passer le CarlaScenarioBridge pour valider sur la physique réelle.
"""

import statistics
from dataclasses import dataclass, field, asdict
from typing import List, Callable, Dict, Optional

from scenario import ScenarioSpec, crossing_scenario, no_pedestrian_scenario
from kinematic_runner import run_kinematic
from ego_controller import CruiseController
from ego_planner import EgoPlanner, PlannerConfig
from metrics import evaluate, MetricsConfig
from intention.base import IntentionPredictor


# ──────────────────────────────────────────────
# DÉFINITION D'UNE SUITE DE SCÉNARIOS (reproductible)
# ──────────────────────────────────────────────

def default_suite(n_crossing: int = 12, n_free: int = 3) -> List[ScenarioSpec]:
    """
    Grille de scénarios standard et reproductible :
      - n_crossing scénarios de traversée (seeds 0..n-1) : le cœur du test
      - n_free scénarios route libre : vérifient qu'on n'est pas trop prudent
    """
    suite = [crossing_scenario(seed=s) for s in range(n_crossing)]
    suite += [no_pedestrian_scenario(seed=s) for s in range(n_free)]
    return suite


# ──────────────────────────────────────────────
# RÉSULTATS AGRÉGÉS PAR MODÈLE
# ──────────────────────────────────────────────

@dataclass
class ModelResult:
    """Métriques agrégées d'un prédicteur sur toute la suite de scénarios."""
    model_name: str
    n_scenarios: int
    rcib_mean: float
    rcib_std: float
    collision_rate: float        # fraction de scénarios avec collision
    goal_rate: float             # fraction de scénarios où le but est atteint
    safety_mean: float
    comfort_mean: float
    success_mean: float
    mean_hard_brakes: float
    mean_min_ttc: float
    per_scenario: List[dict] = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)

    def summary_row(self) -> str:
        return (f"{self.rcib_mean:6.3f} | {self.model_name[:24]:24} | "
                f"{self.collision_rate*100:4.0f}% | {self.goal_rate*100:4.0f}% | "
                f"{self.safety_mean:.2f} {self.comfort_mean:.2f} {self.success_mean:.2f} | "
                f"{self.mean_hard_brakes:5.1f}")


# ──────────────────────────────────────────────
# LE BATCH RUNNER
# ──────────────────────────────────────────────

def evaluate_model(predictor: IntentionPredictor,
                   suite: List[ScenarioSpec],
                   planner_config: Optional[PlannerConfig] = None,
                   metrics_config: Optional[MetricsConfig] = None,
                   runner: Callable = run_kinematic) -> ModelResult:
    """
    Évalue UN prédicteur sur toute la suite et retourne ses métriques agrégées.

    Args:
        predictor      : le prédicteur à évaluer (interface IntentionPredictor)
        suite          : liste de ScenarioSpec (grille reproductible)
        planner_config : réglage du planner réactif (défaut = prudent)
        metrics_config : seuils des métriques (défaut)
        runner         : exécuteur — run_kinematic (défaut) ou un bridge CARLA
    """
    rcibs, safeties, comforts, successes = [], [], [], []
    hard_brakes, min_ttcs = [], []
    collisions = goals = 0
    per_scenario = []

    for spec in suite:
        # reset du prédicteur entre scénarios (utile pour les modèles à état)
        try:
            predictor.reset()
        except Exception:
            pass

        planner = EgoPlanner(predictor, cruise_speed=spec.ego_target_speed,
                             config=planner_config)
        ctrl = CruiseController(target_speed=spec.ego_target_speed)
        trace = runner(spec, controller=ctrl, planner=planner)
        r = evaluate(trace, metrics_config)

        rcibs.append(r.rcib_score)
        safeties.append(r.safety_score)
        comforts.append(r.comfort_score)
        successes.append(r.success_score)
        hard_brakes.append(r.n_hard_brakes)
        min_ttcs.append(r.min_ttc)
        collisions += int(r.collision)
        goals += int(r.reached_goal)
        per_scenario.append({
            "scenario_id": spec.scenario_id,
            "seed": spec.seed,
            "rcib": round(r.rcib_score, 4),
            "collision": r.collision,
            "reached_goal": r.reached_goal,
            "min_ttc": round(r.min_ttc, 3),
            "n_hard_brakes": r.n_hard_brakes,
        })

    n = len(suite)
    return ModelResult(
        model_name=predictor.name,
        n_scenarios=n,
        rcib_mean=statistics.mean(rcibs),
        rcib_std=statistics.pstdev(rcibs) if n > 1 else 0.0,
        collision_rate=collisions / n,
        goal_rate=goals / n,
        safety_mean=statistics.mean(safeties),
        comfort_mean=statistics.mean(comforts),
        success_mean=statistics.mean(successes),
        mean_hard_brakes=statistics.mean(hard_brakes),
        mean_min_ttc=statistics.mean(min_ttcs),
        per_scenario=per_scenario,
    )


def run_benchmark(predictors: List[IntentionPredictor],
                  suite: Optional[List[ScenarioSpec]] = None,
                  planner_config: Optional[PlannerConfig] = None,
                  metrics_config: Optional[MetricsConfig] = None,
                  runner: Callable = run_kinematic) -> List[ModelResult]:
    """
    Évalue tous les prédicteurs sur la même suite et retourne le LEADERBOARD
    (liste de ModelResult triée par RCIB décroissant).
    """
    if suite is None:
        suite = default_suite()

    results = [evaluate_model(p, suite, planner_config, metrics_config, runner)
               for p in predictors]
    results.sort(key=lambda r: r.rcib_mean, reverse=True)
    return results


# ──────────────────────────────────────────────
# AFFICHAGE & SAUVEGARDE DU LEADERBOARD
# ──────────────────────────────────────────────

def format_leaderboard(results: List[ModelResult]) -> str:
    """Formate le leaderboard en tableau lisible."""
    lines = []
    lines.append("=" * 76)
    lines.append(f"{'RCIB':>6} | {'modèle':24} | {'coll':>5} | {'buts':>5} | "
                 f"{'saf/cmf/suc':>14} | freins")
    lines.append("-" * 76)
    for r in results:
        lines.append(r.summary_row())
    lines.append("=" * 76)
    if results:
        n = results[0].n_scenarios
        lines.append(f"({len(results)} modèles sur {n} scénarios — "
                     f"RCIB = 0.5·sécurité + 0.2·confort + 0.3·réussite)")
    return "\n".join(lines)


def save_leaderboard(results: List[ModelResult], path: str = "results/leaderboard.json"):
    """Sauvegarde le leaderboard complet en JSON (reproductibilité / commit)."""
    import json
    from pathlib import Path
    from datetime import datetime
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "generated": datetime.now().isoformat(),
        "n_models": len(results),
        "n_scenarios": results[0].n_scenarios if results else 0,
        "ranking": [r.to_dict() for r in results],
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False, default=str)
    return path
