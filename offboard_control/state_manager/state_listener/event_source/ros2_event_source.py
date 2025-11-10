from offboard_control.domain.model.configuration.ros2_protocol_configuration import Ros2ProtocolConfiguration
from offboard_control.domain.state_listener.i_event_source import IEventSource
from offboard_control.domain.model.configuration.ros2_communication_data_configuration import Ros2CommunicationDataConfiguration

class Ros2EventSource(IEventSource):

    def __init__(
            self, 
            data_configuration: Ros2CommunicationDataConfiguration,
            protocol_configuration: Ros2ProtocolConfiguration
        ):
        self.data_configuration = data_configuration
        self.protocol_configuration = protocol_configuration

    def subscribe(
            self, 
            callback
        ) -> None:

        self.susbscription = self.protocol_configuration.node.create_subscription(
            self.data_configuration.msg_type, #TODO Review class validation
            self.data_configuration.topic,
            callback,
            self.protocol_configuration.qos_profile
        )