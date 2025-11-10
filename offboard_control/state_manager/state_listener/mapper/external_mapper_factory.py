from px4_msgs.msg import VehicleStatus # type: ignore
from px4_msgs.msg import VehicleCommand # type: ignore
from px4_msgs.msg import VehicleGlobalPosition # type: ignore
from px4_msgs.msg import VehicleLocalPosition

from offboard_control.state_manager.state_listener.mapper.px4_vehicle_command_mapper import Px4VehicleCommandMapper
from offboard_control.state_manager.state_listener.mapper.px4_vehicle_status_mapper import Px4VehicleStatusMapper
from offboard_control.state_manager.state_listener.mapper.px4_global_position_mapper import Px4VehicleGlobalPositionMapper
from offboard_control.state_manager.state_listener.mapper.px4_local_position_mapper import Px4VehicleLocalPositionMapper
from offboard_control.domain.mapper.i_external_mapper_factory import IExternalMapperFactory
from offboard_control.domain.mapper.i_external_mapper import IExternalMapper

class ExternalMapperFactory(IExternalMapperFactory):
    def __init__(self):
        pass

    def create(self, external_type: type) -> IExternalMapper:

        if external_type is VehicleStatus:
            return Px4VehicleStatusMapper()
        elif external_type is VehicleCommand:
            return Px4VehicleCommandMapper()
        elif external_type is VehicleGlobalPosition:
            return Px4VehicleGlobalPositionMapper()
        elif external_type is VehicleLocalPosition:
            return Px4VehicleLocalPositionMapper()
        else:
            raise ValueError(f"There is no mapper for this external type: {type(external_type).__name__}")
       

    
        
       