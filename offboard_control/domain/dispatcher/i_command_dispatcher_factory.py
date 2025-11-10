from abc import ABC, abstractmethod
from offboard_control.domain.dispatcher.i_command_dispatcher import ICommandDispatcher
from offboard_control.domain.model.configuration.i_protocol_configuration import IProtocolConfiguration

class ICommandDispatcherFactory(ABC):
    
    @abstractmethod
    def create(
            self, 
            system_id: int,
            autopilot_id: str,
            protocol_configuration: IProtocolConfiguration
        ) -> ICommandDispatcher:
        pass
    
