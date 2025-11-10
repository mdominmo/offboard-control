from px4_msgs.msg import VehicleStatus # type: ignore
from offboard_control.domain.model.state.vehicle_status_state import VehicleStatusState
from offboard_control.domain.mapper.i_external_mapper import IExternalMapper
from offboard_control.domain.constant.vehicle_status_states import VehicleStatusStates

class Px4VehicleStatusMapper(IExternalMapper):

    def map_to(self, domain: VehicleStatusState) -> VehicleStatus:
        pass

    def map_from(self, external: VehicleStatus) -> VehicleStatusState:
        
        state = VehicleStatusState()
        
        state.fail_safe = external.failsafe
        state.arm_state = self.map_arm_state(external.arming_state)
        state.nav_state = self.map_nav_state(external.nav_state)
        return state 
    
    def map_arm_state(self, out: int):
        if out is VehicleStatus.ARMING_STATE_ARMED:
            return VehicleStatusStates.STATE_ARMED
        elif out is VehicleStatus.ARMING_STATE_DISARMED:
            return VehicleStatusStates.STATE_DISARMED
        else:
            return VehicleStatusStates.STATE_ARMED

    def  map_nav_state(self, out: int) -> int:
        
        if out is VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            return VehicleStatusStates.STATE_HOLD
        elif out is VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:    
            return VehicleStatusStates.STATE_TAKEOFF 
        elif out is VehicleStatus.NAVIGATION_STATE_AUTO_RTL:    
            return VehicleStatusStates.STATE_RTL
        elif out is VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return VehicleStatusStates.STATE_OFFBOARD
        else:
            #TODO Create a failed nav state
            return VehicleStatusStates.STATE_HOLD
        
       