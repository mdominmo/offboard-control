from abc import ABC, abstractmethod
from offboard_control.domain.model.i_domain import IDomain

class IRepository(ABC):

    @abstractmethod
    def create(self, new: IDomain) -> None:
        pass
    
    @abstractmethod
    def update(self, new: IDomain) -> None:
        pass
    
    @abstractmethod
    def get(self) -> IDomain:
        pass

    
    
