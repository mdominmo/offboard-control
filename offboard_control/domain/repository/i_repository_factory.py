from abc import ABC, abstractmethod
from offboard_control.domain.repository.i_repository import IRepository

class IRepositoryFactory(ABC):

    @abstractmethod
    def create(self, internal_state: str) -> IRepository:        
        pass    
    
