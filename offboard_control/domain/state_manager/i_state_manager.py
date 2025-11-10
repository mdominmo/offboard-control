from abc import ABC, abstractmethod

class IStateManager(ABC):
 
    def clear_state(self) -> None:
        pass