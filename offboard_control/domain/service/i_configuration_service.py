from abc import ABC, abstractmethod
from offboard_control.domain.model.configuration.offboard_configuration import OffboardConfiguration

class IConfigurationService(ABC):

    abstractmethod
    def get_offboard_configuration(self) -> OffboardConfiguration:
        pass

    @abstractmethod
    def get_offboard_configurations(self) -> list[OffboardConfiguration]:
        pass



    
    
