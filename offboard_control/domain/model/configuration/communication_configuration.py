from offboard_control.domain.model.configuration.i_configuration import IConfiguration
from offboard_control.domain.model.configuration.i_protocol_configuration import IProtocolConfiguration
from offboard_control.domain.model.configuration.communication_data_configuration import CommunicationDataConfiguration
from offboard_control.domain.constant.communication_direction import CommunicationDirection

class CommunicationConfiguration(IConfiguration):
    def __init__(
            self,
            direction: CommunicationDirection,
            communication_data_configuration: dict[str, CommunicationDataConfiguration],
            protocol_configuration: IProtocolConfiguration
        ):
        self.direction = direction
        self.communication_data_configuration = communication_data_configuration
        self.protocol_configuration = protocol_configuration

        
        
        
        
