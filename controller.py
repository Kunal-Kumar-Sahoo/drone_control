from abc import ABC, abstractmethod
import numpy as np


class Controller(ABC):
    @abstractmethod
    def compute_control(self, state, dt):
        pass