from abc import ABC, abstractmethod
from offboard_control.domain.model.configuration.communication_configuration import CommunicationConfiguration
from offboard_control.domain.state_manager.i_state_manager import IStateManager

class IStateManagerAssembler(ABC):
    
    @abstractmethod
    def assemble(self, configurations: list[CommunicationConfiguration]) -> IStateManager:
        pass