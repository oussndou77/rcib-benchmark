# Brancher un vrai prédicteur d'intention dans RCIB

RCIB compare des prédicteurs d'intention via une **interface stable** et une
**frontière de processus**. Vous pouvez brancher n'importe quel modèle — même
incompatible avec l'environnement du simulateur — sans toucher au reste.

## Deux façons de brancher un modèle

### A. Modèle compatible (même Python) → implémenter l'interface directement

Si votre modèle tourne dans le même environnement que le simulateur, sous-classez
`IntentionPredictor` (voir `rcib/intention/heuristic.py` comme exemple) :

```python
from intention.base import IntentionPredictor

class MonModele(IntentionPredictor):
    name = "mon_modele_v1"
    def __init__(self):
        self.model = charger_mon_modele()
    def predict_intent(self, ego, pedestrians):
        return {p.id: float(self.model(...)) for p in pedestrians}
```

### B. Modèle isolé (autre stack) → frontière de processus

C'est le cas de **PIEPredict** (2019, TensorFlow 1.14, Python 3.5) ou de tout modèle
dont les dépendances cassent avec CARLA / le simulateur. Le modèle tourne dans SON
environnement, et RCIB lui parle en JSON via stdin/stdout.

**Étapes :**

1. Copier `runners/predictor_server_example.py` dans l'environnement du modèle.
2. Remplacer la classe `DummyModel` par un wrapper de votre modèle :
   - `__init__` : charger les poids (une seule fois, au démarrage)
   - `predict(ego, pedestrians)` : construire l'entrée attendue par le modèle,
     l'exécuter, convertir la sortie en `{ped_id: score∈[0,1]}`
3. Le protocole ne change pas. Le serveur doit :
   - écrire `{"status": "ready"}` après chargement
   - répondre `{"intents": {...}}` à chaque requête `{"ego":..., "pedestrians":[...]}`

**Côté simulateur**, on branche le modèle isolé ainsi :

```python
from intention.remote import RemoteIntentionPredictor
from ego_planner import EgoPlanner

# Lance le serveur dans l'env isolé (ex. un autre interpréteur Python)
predictor = RemoteIntentionPredictor(
    ["python3.5", "/chemin/predictor_server.py", "--model", "pie"],
    name="PIEPredict",
    startup_timeout=120,   # le chargement TF1 peut être long
)

planner = EgoPlanner(predictor, cruise_speed=8.0)
# ... lancer le scénario avec ce planner ...
predictor.close()
```

## Exemple concret : PIEPredict isolé

PIEPredict prédit la traversée à partir d'une séquence de boîtes englobantes. Pour
l'adapter :

- l'environnement isolé : `conda create -n pie python=3.5 && pip install tensorflow==1.14 keras==2.1`
- le wrapper accumule un historique des positions du piéton (PIE attend une séquence),
  construit le tenseur d'entrée, appelle `model.predict()`, et mappe la probabilité de
  traversée sur `[0,1]`.
- comme RCIB envoie un état par tick, le wrapper maintient lui-même la fenêtre
  temporelle (buffer des N derniers états par `ped_id`).

> Le `reset` du protocole sert exactement à ça : vider le buffer entre deux scénarios.

## Pourquoi ce design

- **Isolation** : les dépendances du modèle (TF1, vieux Python) ne contaminent jamais
  le simulateur ni CARLA.
- **Langage-agnostique** : le serveur peut être en Python, C++, autre — tant qu'il
  parle le protocole JSON.
- **Comparable** : tous les modèles passent par la même interface → le leaderboard
  RCIB les classe équitablement sur des métriques de conduite.
- **Testable** : `predictor_server_example.py` est un serveur fonctionnel sans
  dépendance, qui sert de référence et de test (voir `tests/test_intention.py`).
