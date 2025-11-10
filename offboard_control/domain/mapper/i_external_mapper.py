from abc import ABC, abstractmethod
from offboard_control.domain.model.i_domain import IDomain
class IExternalMapper(ABC):

    @abstractmethod
    def map_from(self, external) -> IDomain:
        pass

    @abstractmethod
    def map_to(self, domain: IDomain):  
        pass