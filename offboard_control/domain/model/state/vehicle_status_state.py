from offboard_control.domain.constant.vehicle_status_states import VehicleStatusStates
from offboard_control.domain.model.state.i_state import IState

class VehicleStatusState(IState):
    def __init__(self):
        
        self._initial_position_received = False
        
        self._initial_lat = -1
        self._initial_lon = -1
        self._initial_alt = -1
        self._lat = -1
        self._lon = -1
        self._alt = -1

        self._nav_state = VehicleStatusStates.STATE_HOLD
        self._arm_state = VehicleStatusStates.STATE_DISARMED
        self._status_failsafe = False
        
        self._arm_count = 0
        

    @property
    def initial_position_received(self):
        return self._initial_position_received

    @property
    def initial_lat(self):
        return self._initial_lat

    @property
    def initial_lon(self):
        return self._initial_lon

    @property
    def initial_alt(self):
        return self._initial_alt
    
    @property
    def nav_state(self):
        return self._nav_state

    @nav_state.setter
    def nav_state(self, value):
        self._nav_state = value
    
    @property
    def arm_state(self):
        return self._arm_state

    @arm_state.setter
    def arm_state(self, value):
        self._arm_state = value

    @property
    def status_failsafe(self):
        return self._status_failsafe

    @status_failsafe.setter
    def status_failsafe(self, value):
        self._status_failsafe = value

    @property
    def lat(self):
        return self._lat

    @lat.setter
    def lat(self, value):
        self._lat = value
        self._set_initial_position_once()

    @property
    def lon(self):
        return self._lon

    @lon.setter
    def lon(self, value):
        self._lon = value
        self._set_initial_position_once()

    @property
    def alt(self):
        return self._alt

    @alt.setter
    def alt(self, value):
        self._alt = value
        self._set_initial_position_once()

    def _set_initial_position_once(self):
        if not self._initial_position_received and self._lat != -1 and self._lon != -1 and self._alt != -1:
            self._initial_lat = self._lat
            self._initial_lon = self._lon
            self._initial_alt = self._alt
            self._initial_position_received = True