from offboard_control.domain.model.configuration.ros2_protocol_configuration import Ros2ProtocolConfiguration
from offboard_control.domain.state_listener.i_event_source import IEventSource
from offboard_control.domain.state_listener.i_event_source_factory import IEventSourceFactory
from offboard_control.state_manager.state_listener.event_source.ros2_event_source import Ros2EventSource
from offboard_control.domain.model.configuration.i_protocol_configuration import IProtocolConfiguration
from offboard_control.domain.model.configuration.communication_data_configuration import CommunicationDataConfiguration

class EventSourceFactory(IEventSourceFactory):

    def create(
            self, 
            data_configuration: CommunicationDataConfiguration,
            protocol_configuration: IProtocolConfiguration
        ) -> IEventSource:

        match protocol_configuration:
            case Ros2ProtocolConfiguration():
                return Ros2EventSource(
                    data_configuration,
                    protocol_configuration
                )
            case __:
                raise ValueError(f"There is no event source for this protocol configuration: {type(protocol_configuration).__name__} ")
            