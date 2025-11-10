from offboard_control.domain.model.state.i_state import IState

class VehicleGlobalPositionState(IState):
    def __init__(self):
        
        self.lat = -1
        self.lon = -1
        self.alt = -1