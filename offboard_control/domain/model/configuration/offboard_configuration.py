from offboard_control.domain.model.configuration.i_configuration import IConfiguration
from offboard_control.domain.model.configuration.communication_configuration import CommunicationConfiguration
from offboard_control.domain.model.configuration.vehicle_configuration import VehicleConfiguration

class OffboardConfiguration(IConfiguration):
    def __init__(
            self,
            incomming_communication_configuration: dict[CommunicationConfiguration],
            outgoing_communication_configuration: dict[CommunicationConfiguration],
            vehicle_configuration: VehicleConfiguration
        ):
        self.incomming_communication_configuration = incomming_communication_configuration
        self.outgoing_communication_configuration = outgoing_communication_configuration
        self.vehicle_configuration = vehicle_configuration
