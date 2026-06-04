#!/usr/bin/env python3
"""
rcib.run_logger — Journalisation des runs pour la reproductibilité (gap n°2).

Sauvegarde tout ce qu'il faut pour rejouer et comparer une run :
  - la config du scénario (seed, carte, version CARLA, modèle d'intention)
  - la trace complète (pour le replay)
  - les métriques calculées (pour le leaderboard)

Tout est en JSON versionnable : commiter results/ dans git = reproductibilité
vérifiable par n'importe qui (ce que le SOTA ne fait pas).
"""

import os
import json
from pathlib import Path
from datetime import datetime
from typing import Optional

from trace import Trace
from metrics import MetricsResult, MetricsConfig


def save_run(trace: Trace, result: MetricsResult,
             output_dir: str = "results",
             config: Optional[MetricsConfig] = None,
             run_name: Optional[str] = None) -> str:
    """
    Sauvegarde une run complète en JSON. Retourne le chemin du fichier.
    """
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    if run_name is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_name = f"{trace.scenario_id}_{trace.intention_model}_{ts}"
    # Nettoyer le nom de fichier
    safe = "".join(c if c.isalnum() or c in "._-" else "_" for c in run_name)
    path = out / f"{safe}.json"

    payload = {
        "run_name": run_name,
        "timestamp": datetime.now().isoformat(),
        "config": config.__dict__ if config else MetricsConfig().__dict__,
        "metrics": result.to_dict(),
        "trace": trace.to_dict(),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False, default=str)
    return str(path)


def load_run(path: str):
    """Recharge une run : retourne (trace, metrics_dict, config_dict)."""
    with open(path, encoding="utf-8") as f:
        payload = json.load(f)
    trace = Trace.from_dict(payload["trace"])
    return trace, payload["metrics"], payload.get("config", {})


def build_leaderboard(output_dir: str = "results") -> list:
    """
    Agrège toutes les runs d'un dossier en un classement par score RCIB.
    C'est la vue 'leaderboard' qui compare les modèles d'intention.
    """
    out = Path(output_dir)
    rows = []
    for fp in out.glob("*.json"):
        try:
            with open(fp, encoding="utf-8") as f:
                p = json.load(f)
            m = p.get("metrics", {})
            tr = p.get("trace", {})
            rows.append({
                "run": p.get("run_name", fp.stem),
                "scenario": tr.get("scenario_id", "?"),
                "intention_model": tr.get("intention_model", "?"),
                "rcib_score": m.get("rcib_score", 0.0),
                "collision": m.get("collision", None),
                "min_ttc": m.get("min_ttc", None),
                "max_decel": m.get("max_decel", None),
                "reached_goal": m.get("reached_goal", None),
            })
        except (json.JSONDecodeError, KeyError):
            continue
    rows.sort(key=lambda r: r["rcib_score"], reverse=True)
    return rows


def print_leaderboard(output_dir: str = "results") -> None:
    """Affiche le leaderboard en tableau lisible."""
    rows = build_leaderboard(output_dir)
    if not rows:
        print("(aucune run dans", output_dir, ")")
        return
    print(f"\n{'='*78}")
    print(f"{'RCIB':>6} | {'model':20} | {'scenario':18} | {'coll':4} | {'TTC':>5} | goal")
    print(f"{'-'*78}")
    for r in rows:
        coll = "yes" if r["collision"] else "no"
        ttc = f"{r['min_ttc']:.1f}" if r["min_ttc"] is not None else "?"
        goal = "✓" if r["reached_goal"] else "✗"
        print(f"{r['rcib_score']:6.3f} | {r['intention_model'][:20]:20} | "
              f"{r['scenario'][:18]:18} | {coll:4} | {ttc:>5} | {goal}")
    print(f"{'='*78}\n")
