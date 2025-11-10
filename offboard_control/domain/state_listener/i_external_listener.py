from abc import ABC, abstractmethod

class IExternalListener(ABC):

    @abstractmethod
    def event_handler(self, external) -> None:
        pass

        
       