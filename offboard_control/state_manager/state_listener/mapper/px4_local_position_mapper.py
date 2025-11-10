from px4_msgs.msg import VehicleLocalPosition # type: ignore
from offboard_control.domain.model.state.vehicle_local_position_state import VehicleLocalPositionState
from offboard_control.domain.mapper.i_external_mapper import IExternalMapper

class Px4VehicleLocalPositionMapper(IExternalMapper):

    def map_to(self, domain: VehicleLocalPositionState) -> VehicleLocalPosition:
        pass

    def map_from(self, external: VehicleLocalPosition) -> VehicleLocalPositionState:
        
        state = VehicleLocalPositionState()
        
        state.x = external.x
        state.y = external.y
        state.z = external.z

        state.vx = external.vx
        state.vy = external.vy
        state.vz = external.vz

        state.ref_lat = external.ref_lat
        state.ref_lon = external.ref_lon
        state.ref_alt = external.ref_alt

        state.heading = external.heading

        return state 