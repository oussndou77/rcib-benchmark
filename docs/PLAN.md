# RCIB — Plan de construction du Sim Runner CARLA

**Reactive Closed-loop Intention Benchmark**
Document de planification — à lire entièrement avant d'écrire du code.

Ce document est aussi le socle de la doc portfolio/GitHub : il explique *pourquoi*
le benchmark existe (le gap consensus), *quoi* on construit, et *comment*, en
anticipant les pièges d'intégration réels.

---

## 0. Le pourquoi (rappel du gap consensus)

Le swarm de recherche a analysé 37 sources et a fait émerger, de façon robuste et
répétée, un gap n°1 corroboré par **10 sources indépendantes** :

> Les modèles de prédiction d'intention piéton sont évalués en **open-loop**
> (ADE/FDE/MR) — jamais sur leur impact réel sur la **conduite** (collisions, freinages).

RCIB comble ce trou : il branche un prédicteur d'intention dans un simulateur
**closed-loop** (CARLA) et mesure des métriques de **conduite**, pas de prédiction.
C'est la contribution différenciante et vendable.

---

## 1. La contrainte qui façonne TOUT : l'incompatibilité des environnements

C'est le point le plus important du plan. Les briques ne peuvent PAS cohabiter :

| Brique | Stack | Époque |
|--------|-------|--------|
| CARLA 0.9.15 (client Python) | Python 3.7+, son propre `carla` wheel | 2023 |
| PIEPredict | **TensorFlow 1.9/1.14, Keras 2.1, Python 3.5, CUDA 9** | 2019 |
| Trajectron++ | PyTorch 1.x, Python 3.6 | 2020 |

**On ne peut pas faire `pip install carla tensorflow==1.14` dans le même venv.** C'est
incompatible (Python, CUDA, dépendances). Toute architecture naïve qui essaie ça
échoue immédiatement.

### La solution : découplage par processus (l'Intention Adapter est un service)

Au lieu d'importer le prédicteur dans le client CARLA, on l'isole derrière une
**frontière de processus**. Trois options, par ordre de robustesse :

1. **Service local (recommandé pour démarrer)** : le prédicteur tourne dans SON propre
   conteneur/venv et expose un endpoint (HTTP local ou socket). Le client CARLA lui
   envoie l'état des piétons, reçoit l'intention. Découplage total des dépendances.
2. **Modèle ré-implémenté/exporté** : convertir le prédicteur en ONNX (format neutre)
   et l'exécuter via `onnxruntime` dans l'environnement CARLA. Élimine TF1 mais demande
   une conversion (pas toujours triviale pour TF1).
3. **Baseline ré-implémentée** : pour la V0, ré-implémenter une heuristique d'intention
   simple (vitesse + orientation + position vs passage piéton) directement en Python
   moderne. Pas de dépendance externe, valide toute la chaîne, et sert de point de
   comparaison honnête pour les vrais modèles.

**Décision pour la V0 : on commence par l'option 3 (baseline), puis option 1 (service)
pour PIEPredict.** Ça permet de valider toute la boucle closed-loop SANS se battre
contre TF1 dès le départ. On branche les vrais modèles une fois la mécanique prouvée.

C'est exactement la philosophie qu'on a suivie pour tout le reste : valider le squelette
d'abord, brancher le lourd ensuite.

---

## 2. Choix de version CARLA (le piège n°1)

**Le piège documenté** : le serveur CARLA (Docker) et le package Python `carla`
DOIVENT avoir la même version, sinon erreurs RPC à coup sûr.

| Branche | Image | Moteur | Verdict |
|---------|-------|--------|---------|
| 0.9.15 | `carlasim/carla:0.9.15` | UE4 | **Recommandé** — mature, stable, doc abondante, package `carla` sur PyPI |
| 0.10.0 | `carlasim/carla:0.10.0` | UE5 | Récent, plus lourd, moins de retours d'expérience |

**Décision : CARLA 0.9.15.** On verrouille la version partout :
- serveur : `carlasim/carla:0.9.15`
- client : `pip install carla==0.9.15`

Drapeau headless (pas d'écran sur RunPod) : `-RenderOffScreen`. Si pas de GPU pour le
rendu : `-nullrhi` (mais on a un GPU sur RunPod, donc `-RenderOffScreen` suffit).

---

## 3. Le compute (RunPod)

CARLA 0.9.15 (UE4) tourne confortablement sur un GPU type **RTX 3090 / A4000 / A5000**
(8 Go VRAM minimum, 12+ recommandé). Inutile de prendre un A100 pour la V0 — CARLA
n'est pas un entraînement, c'est du rendu + simulation.

**Disque** : prévoir ~30 Go pour l'image CARLA + le workspace. (Le dataset PIE complet =
1.1 To, mais on n'en a PAS besoin pour le closed-loop : on génère les scénarios DANS
CARLA. Le dataset PIE ne sert qu'à ré-entraîner PIEPredict, ce qu'on ne fait pas.)

**Stratégie coût** : on développe et teste localement tout ce qui peut l'être sans GPU
(le Metrics Harness, la logique de scénario en mock), et on ne réserve le pod RunPod
que pour les runs CARLA réels. On éteint le pod entre les sessions.

---

## 4. Architecture détaillée des composants

### 4.1 Scenario Bridge
Responsabilité : peupler le monde CARLA et jouer un scénario reproductible.
- connecte au serveur (`carla.Client('localhost', 2000)`)
- charge une carte (ex. Town01, légère)
- spawn l'ego (véhicule) à un point de départ fixe
- spawn N piétons (walkers) avec des trajectoires de traversée scriptées
- fixe une seed pour la reproductibilité (même scénario à chaque run)
- tourne en mode synchrone (`world.tick()`) pour un contrôle déterministe

Interface de sortie (par tick) : positions/vitesses ego + piétons, horodatage.

### 4.2 Intention Adapter (frontière de processus — voir §1)
Responsabilité : à partir de l'état des piétons, prédire l'intention de traverser.
- **V0 (baseline)** : heuristique pure Python — un piéton est "intentionnel" si sa
  vitesse projetée le rapproche de la chaussée / d'un passage. Zéro dépendance.
- **V1 (service)** : client léger qui envoie l'état à un service PIEPredict isolé
  (HTTP local) et reçoit un score d'intention.
- Interface stable : `predict_intent(pedestrian_states) -> {ped_id: intent_score}`.
  Peu importe l'implémentation derrière, l'Ego Planner ne voit que cette interface.

### 4.3 Ego Planner
Responsabilité : décider l'action de l'ego en fonction de l'intention prédite.
- V0 : contrôleur réactif simple — si un piéton à intention élevée est dans le couloir
  de l'ego et proche, freiner proportionnellement à la distance/TTC.
- applique le contrôle via l'API CARLA (`vehicle.apply_control(throttle/brake)`).
- C'est CE composant qui rend l'évaluation *closed-loop* : l'action change l'état suivant.

### 4.4 Metrics Harness (cœur scientifique, testable SANS GPU)
Responsabilité : transformer une trace de simulation en scores de conduite.
- **collision** : binaire + gravité (vitesse à l'impact)
- **time-to-collision (TTC)** : min sur la trace, alerte si < seuil
- **comfort** : jerk/décélérations brusques (freinages d'urgence)
- **task success** : l'ego atteint-il sa destination sans incident ?
- **score composite RCIB** : la métrique unifiée que le swarm réclame (reco n°4) —
  combine sécurité + confort + réussite en un score comparable entre modèles.

Pourquoi testable sans GPU : il prend en entrée une **trace** (liste d'états au cours
du temps). On peut lui donner des traces synthétiques pour le valider entièrement hors
CARLA. C'est pour ça qu'on le construit en premier.

### 4.5 Run Logger
Responsabilité : reproductibilité (le gap n°2).
- sauvegarde la config du run (seed, carte, scénario, modèle d'intention, version CARLA)
- sauvegarde la trace complète + les scores en JSON
- permet le replay et la comparaison entre modèles (tableau de leaderboard)

---

## 5. Ordre de construction (chaque phase validée avant la suivante)

| Phase | Quoi | GPU ? | Critère de validation |
|-------|------|-------|----------------------|
| **0** | Squelette repo + Dockerfile RunPod + smoke test | oui (court) | CARLA démarre headless, le client s'y connecte, `world.tick()` tourne |
| **1** | Metrics Harness + baseline intention | **non** | scores corrects sur traces synthétiques (tests unitaires) |
| **2** | Scenario Bridge | oui | piétons + ego spawnent, scénario reproductible (même seed → même trace) |
| **3** | Ego Planner réactif + boucle closed-loop | oui | l'ego freine face à un piéton ; collision évitée vs baseline passive |
| **4** | Intention Adapter en service (PIEPredict isolé) | oui | le vrai modèle remplace la baseline via l'interface stable |
| **5** | Batch runner + leaderboard + doc | oui | N scénarios × M modèles → tableau de scores RCIB reproductible |

À la fin de la Phase 3, tu as déjà un **résultat publiable** : "voici un benchmark
closed-loop pour l'intention piéton, et voici l'écart mesuré entre un ego réactif et
un ego passif". Les phases 4-5 ajoutent les vrais modèles SOTA et l'échelle.

---

## 6. Structure du repo (portfolio + GitHub dès le départ)

```
rcib-benchmark/
├── README.md                  # le pitch : le gap, la contribution, comment lancer
├── docker/
│   └── Dockerfile.runpod      # CARLA 0.9.15 + client, prêt pour RunPod
├── rcib/
│   ├── scenario_bridge.py     # Phase 2
│   ├── intention/
│   │   ├── base.py            # interface predict_intent (stable)
│   │   ├── heuristic.py       # baseline V0 (Phase 1)
│   │   └── pie_service.py     # client du service PIEPredict (Phase 4)
│   ├── ego_planner.py         # Phase 3
│   ├── metrics.py             # Metrics Harness (Phase 1)
│   └── run_logger.py          # Phase 1
├── runners/
│   ├── smoke_test.py          # Phase 0
│   └── batch_eval.py          # Phase 5
├── tests/
│   └── test_metrics.py        # validation hors-GPU du harness
├── results/                   # traces + scores JSON (versionnés = reproductibilité)
└── docs/
    └── PLAN.md                # ce document
```

Tout est pensé pour être **montrable** : un recruteur EPFL/industrie clone le repo,
lit le README, comprend le gap et la contribution, et peut relancer le benchmark.

---

## 7. Pièges anticipés (la liste qui évite de brûler du GPU pour rien)

1. **Version mismatch CARLA** → verrouillée à 0.9.15 partout (§2). Vérifier au smoke test.
2. **PIEPredict incompatible** → isolé en service, jamais dans le venv CARLA (§1).
3. **Mode asynchrone non déterministe** → toujours `world.tick()` synchrone + seed fixe.
4. **Headless sur RunPod** → `-RenderOffScreen`, jamais de fenêtre.
5. **Pod qui tourne pour rien** → développer hors-GPU ce qui peut l'être ; éteindre le pod.
6. **Dataset PIE 1.1 To** → PAS nécessaire ; on génère les scénarios dans CARLA.
7. **Métriques non comparables** (ADE 9.0 vs RMSE 1.0, le gap relevé) → le score RCIB
   composite normalise tout sur une échelle de conduite commune.

---

## 8. Ce qui rend ce livrable vendable

- **Comble un gap réel** confirmé par 10 sources indépendantes du SOTA.
- **Reproductible** (seed, versions verrouillées, traces versionnées) — ce que la
  recherche actuelle ne fait pas (gap reproductibilité).
- **Métrique orientée déploiement** (collision/TTC/comfort) — ce qui parle à un Waymo,
  un Mobileye, un NVIDIA, pas l'ADE/FDE académique.
- **Extensible** : brancher un nouveau prédicteur = implémenter une interface
  `predict_intent`. Le benchmark devient un leaderboard.
- **Aligné EPFL VITA** : prédiction piéton + évaluation rigoureuse = le cœur du lab d'Alahi.
```
