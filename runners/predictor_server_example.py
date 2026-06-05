#!/usr/bin/env python3
"""
predictor_server_example.py — SQUELETTE d'un serveur de prédiction d'intention.

C'est le côté "modèle isolé" de la frontière de processus (cf. rcib/intention/remote.py).
Il montre comment envelopper N'IMPORTE QUEL modèle pour qu'il parle le protocole RCIB.
Le simulateur lance ce script comme sous-processus et lui parle en JSON sur stdin/stdout.

IMPORTANT : ce fichier est volontairement SANS dépendance (juste la lib standard) pour
servir de référence exécutable et de test. Pour brancher un vrai modèle (PIEPredict,
Trajectron++), on copie ce squelette dans l'environnement isolé du modèle (Python 3.5
+ TF1.14 pour PIEPredict, par ex.), et on remplace `DummyModel.predict` par l'appel au
vrai modèle. Le protocole, lui, ne change pas.

Protocole (une ligne JSON par message) :
  -> au démarrage, le serveur écrit : {"status": "ready"}
  -> requête du simulateur          : {"ego": {...}, "pedestrians": [{...}]}
  <- réponse du serveur             : {"intents": {"walker_01": 0.83, ...}}
  -> commandes spéciales            : {"command": "reset"} | {"command": "shutdown"}

Lancement (test local) :
    python3 predictor_server_example.py
    puis taper :  {"ego":{"id":"ego","x":0,"y":0,"vx":8,"vy":0},"pedestrians":[{"id":"w1","x":10,"y":2,"vx":0,"vy":-1}]}
"""

import sys
import json
import math


class DummyModel:
    """
    Modèle jouet : reproduit une logique d'intention simple (proximité + approche).
    REMPLACER par un vrai modèle dans l'environnement isolé.

    Pour PIEPredict (exemple) :
        - dans __init__ : charger le graphe TF1.14 et les poids
        - dans predict  : construire le tenseur d'entrée attendu par PIE à partir de
          l'observation, lancer sess.run(...), convertir la sortie en score [0,1]
    """

    def __init__(self):
        # Ici : charger les poids du vrai modèle (one-time, au démarrage).
        self.lateral_scale = 4.0

    def predict(self, ego: dict, pedestrians: list) -> dict:
        """ego/pedestrians : dicts {id,x,y,vx,vy}. Retourne {ped_id: score}."""
        speed = math.hypot(ego["vx"], ego["vy"])
        if speed < 0.1:
            dx, dy = 1.0, 0.0
        else:
            dx, dy = ego["vx"] / speed, ego["vy"] / speed
        nx, ny = -dy, dx

        out = {}
        for p in pedestrians:
            rx, ry = p["x"] - ego["x"], p["y"] - ego["y"]
            longitudinal = rx * dx + ry * dy
            if longitudinal <= 0:
                out[p["id"]] = 0.0
                continue
            lateral = rx * nx + ry * ny
            prox = math.exp(-abs(lateral) / self.lateral_scale)
            v_toward = -(p["vx"] * nx + p["vy"] * ny) * (1 if lateral > 0 else -1)
            approach = min(1.0, max(0.0, v_toward) / 2.0)
            out[p["id"]] = max(0.0, min(1.0, prox * (0.4 + 0.6 * approach)))
        return out

    def reset(self):
        """Réinitialiser l'état interne entre scénarios (si le modèle en a)."""
        pass


def main():
    model = DummyModel()

    # Handshake : signaler qu'on est prêt (après chargement du modèle)
    sys.stdout.write(json.dumps({"status": "ready"}) + "\n")
    sys.stdout.flush()

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            sys.stdout.write(json.dumps({"error": "invalid json"}) + "\n")
            sys.stdout.flush()
            continue

        # Commandes spéciales
        cmd = msg.get("command")
        if cmd == "shutdown":
            break
        if cmd == "reset":
            model.reset()
            sys.stdout.write(json.dumps({"status": "reset"}) + "\n")
            sys.stdout.flush()
            continue

        # Requête de prédiction
        ego = msg.get("ego")
        peds = msg.get("pedestrians", [])
        if ego is None:
            sys.stdout.write(json.dumps({"error": "missing ego"}) + "\n")
            sys.stdout.flush()
            continue

        try:
            intents = model.predict(ego, peds)
        except Exception as e:
            sys.stdout.write(json.dumps({"error": str(e)}) + "\n")
            sys.stdout.flush()
            continue

        sys.stdout.write(json.dumps({"intents": intents}) + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
