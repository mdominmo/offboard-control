from offboard_control.domain.repository.i_repository import IRepository
from offboard_control.domain.repository.i_repository_factory import IRepositoryFactory
from offboard_control.domain.model.state.vehicle_status_state import VehicleStatusState
from offboard_control.domain.model.state.vehicle_global_position_state import VehicleGlobalPositionState
from offboard_control.domain.model.state.vehicle_local_position_state import VehicleLocalPositionState
from offboard_control.repository.state_repository import StateRepository
from offboard_control.domain.constant.states import States
from offboard_control.domain.model.state.operation_status_state import OperationStatusState

class StateRepositoryFactory(IRepositoryFactory):

    def create(self, internal_state: str) -> IRepository:
        
        if internal_state is States.STATUS:
            return StateRepository(VehicleStatusState())
        elif internal_state is States.GLOBAL_POSITION:
                return StateRepository(VehicleGlobalPositionState())
        elif internal_state is States.LOCAL_POSITION:
                return StateRepository(VehicleLocalPositionState())
        elif internal_state is States.OPERATION_STATUS:
                return StateRepository(OperationStatusState())
        else:
            raise ValueError(f"There is no state repository for this internal state: {internal_state}")