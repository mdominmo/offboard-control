from offboard_control.domain.state_manager.i_state_manager import IStateManager
from offboard_control.domain.state_listener.i_external_listener import IExternalListener
from offboard_control.repository.state_repository import StateRepository

class StateManager(IStateManager):
    def __init__(
            self,
            state_listeners: list[IExternalListener],
            state_repositories: dict[StateRepository]
        ):
        self.state_listeners = state_listeners
        self.state_repositories = state_repositories
    
    def clear_state(self) -> None:
        #TODO
        pass;