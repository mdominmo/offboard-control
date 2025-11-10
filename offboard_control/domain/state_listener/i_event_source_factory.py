from abc import ABC, abstractmethod
from offboard_control.domain.model.configuration.communication_configuration import CommunicationConfiguration
from offboard_control.domain.state_listener.i_event_source import IEventSource
from offboard_control.domain.model.configuration.i_protocol_configuration import IProtocolConfiguration

class IEventSourceFactory(ABC):

    @abstractmethod
    def create(self, configuration: IProtocolConfiguration) -> IEventSource:
        pass

        
       