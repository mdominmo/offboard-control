from offboard_control.domain.constant.vehicle_status_states import VehicleStatusStates
from offboard_control.domain.model.state.i_state import IState
from offboard_control.domain.constant.operation_status import OperationStatus

class OperationStatusState(IState):
    def __init__(
            self
        ):
        self.operation_state = OperationStatus.RUNNING
        self.operation_state = OperationStatus.CANCELED
        # self.operation_state = OperationStatus.WAITING_FOR_COMMAND