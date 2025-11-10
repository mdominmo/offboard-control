from offboard_control.domain.dispatcher.i_command_dispatcher import ICommandDispatcher
from offboard_control.dispatcher.px4_command_dispatcher import Px4CommandDispatcher
from offboard_control.domain.dispatcher.i_command_dispatcher_factory import ICommandDispatcherFactory
from offboard_control.domain.model.configuration.i_protocol_configuration import IProtocolConfiguration
from offboard_control.domain.constant.autopilot_id import AutopilotId
from offboard_control.domain.model.configuration.communication_configuration import CommunicationConfiguration

class CommandDispatcherFactory(ICommandDispatcherFactory):

    def create(
            self, 
            vehicle_id: int,
            autopilot_id: str,
            communication_configuration: CommunicationConfiguration  
        ) -> ICommandDispatcher:
        match autopilot_id:
            case AutopilotId.PX4:
                return Px4CommandDispatcher(
                    vehicle_id, 
                    communication_configuration
                )   
            case _:
                raise ValueError(f"There is no command dispatcher for this autopilot_id: {autopilot_id}")