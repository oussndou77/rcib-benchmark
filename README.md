# RCIB — Reactive Closed-loop Intention Benchmark

Un benchmark **closed-loop** pour les modèles de prédiction d'intention piéton en
conduite autonome. Là où le SOTA mesure la qualité de prédiction en *open-loop*
(ADE/FDE/Miss-Rate), RCIB mesure ce qui compte pour le déploiement : **l'impact d'un
prédicteur d'intention sur la conduite** — collisions, marge de sécurité, confort.

## Pourquoi

Une analyse systématique de la littérature (37 sources : repos + papers) fait ressortir
un gap confirmé par **10 sources indépendantes** : les prédicteurs d'intention piéton
sont évalués isolément, jamais sur leur effet réel sur les décisions de conduite. RCIB
comble ce trou.

## Ce que ça mesure

| Métrique | Sens |
|----------|------|
| `collision` + gravité | contact ? vitesse à l'impact |
| `min_ttc` | marge de sécurité minimale (time-to-collision) |
| `max_decel`, `rms_jerk` | confort (freinages brusques, à-coups) |
| `n_hard_brakes` | nombre de freinages d'urgence |
| `reached_goal` | l'ego atteint-il sa destination ? |
| **`rcib_score`** | **score composite [0,1] : sécurité + confort + réussite** |

Le score composite résout aussi l'incomparabilité des métriques actuelles (ADE 9.0 vs
RMSE 1.0 vs mAP 0.28 ne se comparent pas) en ramenant tout sur une échelle de conduite.

## Statut (construction par phases)

- [x] **Phase 1 — Metrics Harness** : cœur scientifique, pur Python, testé. Tourne sans GPU.
- [x] **Phase 0 — smoke test CARLA sur RunPod** : VALIDÉ sur RTX 3090 (client 0.9.15 ↔ serveur 0.9.15, connexion RPC OK). Setup automatisé : `runners/setup_runpod.sh`. Voir `runners/RUNPOD_GUIDE.md`.
- [ ] Phase 2 — Scenario Bridge (spawn piétons + ego dans CARLA)
- [ ] Phase 3 — Ego Planner réactif + boucle closed-loop
- [ ] Phase 4 — Intention Adapter en service (PIEPredict / Trajectron++ isolés)
- [ ] Phase 5 — Batch runner + leaderboard multi-modèles

## Architecture (clé : découplage du prédicteur)

Le prédicteur d'intention est isolé derrière une interface stable `predict_intent()`.
Cela permet de brancher n'importe quel modèle (heuristique, PIEPredict en TF1,
Trajectron++ en PyTorch, ONNX) sans toucher au reste — et de tous les comparer dans
le leaderboard. Voir `docs/PLAN.md`.

## Lancer les tests (sans GPU)

```bash
cd rcib
python ../tests/test_metrics.py
```

## Exemple : réactif vs passif

Le test fondateur du benchmark — un ego qui utilise l'intention prédite pour freiner
vs un ego qui l'ignore :

```
PASSIF  (ignore l'intention) : RCIB=0.200 | COLLISION@10.0m/s
RÉACTIF (utilise l'intention): RCIB=0.598 | no-collision
→ gain RCIB : +0.398
```

Le benchmark distingue quantitativement un bon comportement d'un mauvais, sur des
métriques de conduite — pas de prédiction.

## Structure

```
rcib/
├── trace.py            # modèle de données d'une run (contrat bridge↔harness)
├── metrics.py          # le Metrics Harness (TTC, collision, confort, score RCIB)
├── intention/
│   ├── base.py         # interface stable IntentionPredictor
│   └── heuristic.py    # baseline V0 (pure Python)
└── run_logger.py       # sauvegarde + leaderboard (reproductibilité)
tests/test_metrics.py   # validation hors-GPU
docs/PLAN.md            # plan complet + pièges anticipés
```
