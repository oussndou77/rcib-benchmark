#!/usr/bin/env python3
"""
rcib.intention.remote — Adaptateur de prédiction "frontière de processus".

CŒUR DE LA PHASE 4. Permet de brancher un prédicteur SOTA qui tourne dans un
ENVIRONNEMENT ISOLÉ (autre version de Python, TF1.14, PyTorch, conteneur séparé)
sans rien changer au simulateur. C'est la solution au problème central du PLAN :
PIEPredict (2019, TF1.14, Python 3.5) est incompatible avec CARLA moderne et notre
Python — donc le prédicteur ne s'IMPORTE pas, il tourne à côté et on lui PARLE.

Architecture :
    [Simulateur / EgoPlanner]                 [Processus prédicteur isolé]
    RemoteIntentionPredictor  --JSON stdin-->  predictor_server (votre modèle)
         (implémente             <--JSON stdout--      Python 3.5 / TF1 / PyTorch...
          IntentionPredictor)

Protocole (une ligne JSON par échange, robuste et sans dépendance réseau) :
    requête  : {"ego": {...}, "pedestrians": [{...}, ...]}
    réponse  : {"intents": {"walker_01": 0.83, ...}}

Le simulateur ne voit qu'un IntentionPredictor standard. Le modèle réel n'a qu'à
implémenter le côté serveur (voir runners/predictor_server_example.py) — il peut être
écrit dans n'importe quel langage/stack tant qu'il lit/écrit ce protocole.

Pourquoi stdin/stdout JSON et pas HTTP ? Zéro dépendance, zéro gestion de port, et
ça marche identiquement en local et dans un conteneur. Un backend HTTP peut être ajouté
plus tard pour un déploiement type model-serving (cf. ARIA), via la même interface.
"""

import json
import subprocess
import time
from typing import Dict, List, Optional

from trace import AgentState
from intention.base import IntentionPredictor


class PredictorProtocolError(RuntimeError):
    """Erreur de communication avec le processus prédicteur."""


def _agent_to_dict(a: AgentState) -> dict:
    return {"id": a.id, "x": a.x, "y": a.y, "vx": a.vx, "vy": a.vy}


class RemoteIntentionPredictor(IntentionPredictor):
    """
    Délègue la prédiction à un processus externe via JSON sur stdin/stdout.

    Le processus est lancé au premier appel (lazy) et réutilisé ensuite (un seul
    démarrage du modèle, pas un par tick). Il est arrêté proprement par close().
    """

    def __init__(self, command: List[str], name: Optional[str] = None,
                 startup_timeout: float = 60.0, call_timeout: float = 10.0):
        """
        command         : commande pour lancer le serveur prédicteur,
                          ex. ["python3.5", "predictor_server.py", "--model", "pie"]
        name            : nom affiché du modèle (sinon dérivé de la commande)
        startup_timeout : s — temps laissé au modèle pour démarrer (chargement poids)
        call_timeout    : s — temps max pour une prédiction
        """
        self.command = command
        self.name = name or f"remote[{command[-1] if command else 'unknown'}]"
        self.startup_timeout = startup_timeout
        self.call_timeout = call_timeout
        self.proc: Optional[subprocess.Popen] = None
        self._ready = False

    # ──────────────────────────────────────────
    def _ensure_started(self):
        """Lance le processus prédicteur s'il ne tourne pas encore."""
        if self.proc is not None and self.proc.poll() is None:
            return
        try:
            self.proc = subprocess.Popen(
                self.command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,   # ligne par ligne
            )
        except FileNotFoundError as e:
            raise PredictorProtocolError(
                f"Impossible de lancer le prédicteur {self.command!r}: {e}")

        # Attendre le handshake "ready" (le serveur écrit {"status": "ready"})
        deadline = time.time() + self.startup_timeout
        while time.time() < deadline:
            line = self.proc.stdout.readline()
            if not line:
                if self.proc.poll() is not None:
                    err = self.proc.stderr.read()
                    raise PredictorProtocolError(
                        f"Le prédicteur s'est arrêté au démarrage. stderr:\n{err}")
                continue
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                # Le serveur peut logger du texte avant le handshake : on ignore.
                continue
            if msg.get("status") == "ready":
                self._ready = True
                return
        raise PredictorProtocolError(
            f"Le prédicteur n'a pas signalé 'ready' en {self.startup_timeout}s")

    # ──────────────────────────────────────────
    def predict_intent(self, ego: AgentState,
                       pedestrians: List[AgentState]) -> Dict[str, float]:
        self._ensure_started()

        request = {
            "ego": _agent_to_dict(ego),
            "pedestrians": [_agent_to_dict(p) for p in pedestrians],
        }
        try:
            self.proc.stdin.write(json.dumps(request) + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError) as e:
            raise PredictorProtocolError(f"Écriture vers le prédicteur échouée: {e}")

        # Lire la réponse (une ligne JSON)
        line = self.proc.stdout.readline()
        if not line:
            err = self.proc.stderr.read() if self.proc.stderr else ""
            raise PredictorProtocolError(
                f"Pas de réponse du prédicteur. stderr:\n{err}")
        try:
            resp = json.loads(line.strip())
        except json.JSONDecodeError as e:
            raise PredictorProtocolError(f"Réponse non-JSON du prédicteur: {line!r} ({e})")

        intents = resp.get("intents", {})
        # Garantir des floats bornés dans [0,1], et une entrée par piéton observé
        out: Dict[str, float] = {}
        for p in pedestrians:
            v = intents.get(p.id, 0.0)
            try:
                v = float(v)
            except (TypeError, ValueError):
                v = 0.0
            out[p.id] = max(0.0, min(1.0, v))
        return out

    # ──────────────────────────────────────────
    def reset(self) -> None:
        """Signale au serveur un nouveau scénario (optionnel côté serveur)."""
        if self.proc is not None and self.proc.poll() is None:
            try:
                self.proc.stdin.write(json.dumps({"command": "reset"}) + "\n")
                self.proc.stdin.flush()
                self.proc.stdout.readline()  # accusé (ignoré)
            except (BrokenPipeError, OSError):
                pass

    def close(self) -> None:
        """Arrête proprement le processus prédicteur."""
        if self.proc is not None:
            try:
                self.proc.stdin.write(json.dumps({"command": "shutdown"}) + "\n")
                self.proc.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None
            self._ready = False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
