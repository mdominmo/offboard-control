from abc import ABC, abstractmethod
from offboard_control.domain.mapper.i_external_mapper import IExternalMapper

class IExternalMapperFactory(ABC):

    @abstractmethod
    def create(self, external_state: type) -> IExternalMapper:
        pass