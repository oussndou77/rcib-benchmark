#!/usr/bin/env python3
"""
rcib.intention.base — Interface STABLE de prédiction d'intention.

C'est le contrat qui découple le prédicteur du reste du système (voir PLAN §1).
Peu importe l'implémentation derrière (heuristique pure Python, service PIEPredict
isolé en TF1, Trajectron++ en PyTorch, modèle ONNX) : l'Ego Planner ne voit QUE
cette interface. C'est ce qui permet de brancher n'importe quel modèle SOTA sans
toucher au reste, et de comparer les modèles dans le leaderboard.
"""

from abc import ABC, abstractmethod
from typing import Dict, List
from trace import AgentState


class IntentionPredictor(ABC):
    """
    Prédit, pour chaque piéton observé, un score d'intention de traverser dans [0,1].
    0 = ne va pas traverser, 1 = va traverser de façon imminente.
    """

    name: str = "base"

    @abstractmethod
    def predict_intent(self, ego: AgentState,
                       pedestrians: List[AgentState]) -> Dict[str, float]:
        """
        Args:
            ego          : état courant de l'ego
            pedestrians  : états courants des piétons observés
        Returns:
            dict {pedestrian_id: intent_score in [0,1]}
        """
        ...

    def reset(self) -> None:
        """Réinitialise l'état interne (entre deux scénarios). Optionnel."""
        pass
