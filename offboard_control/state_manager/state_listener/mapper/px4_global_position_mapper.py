from px4_msgs.msg import VehicleGlobalPosition # type: ignore
from offboard_control.domain.model.state.vehicle_global_position_state import VehicleGlobalPositionState
from offboard_control.domain.mapper.i_external_mapper import IExternalMapper

class Px4VehicleGlobalPositionMapper(IExternalMapper):

    def map_to(self, domain: VehicleGlobalPositionState) -> VehicleGlobalPosition:
        pass

    def map_from(self, external: VehicleGlobalPosition) -> VehicleGlobalPositionState:
        
        state = VehicleGlobalPositionState()
        
        state.lat = external.lat
        state.lon = external.lon
        state.alt = external.alt 

        return state 