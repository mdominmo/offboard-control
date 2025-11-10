from abc import ABC, abstractmethod
from offboard_control.domain.model.configuration.i_protocol_configuration import IProtocolConfiguration
class IEventSource(ABC):

    @abstractmethod
    def subscribe(
        self, 
        callback
    ) -> None:
        pass

        
       