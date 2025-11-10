import threading
from offboard_control.domain.repository.i_repository import IRepository
from offboard_control.domain.model.state.i_state import IState

class StateRepository(IRepository):

    def __init__(self, state: IState):
        self._state = state
        self._lock = threading.Lock()

    def create(self, new_state : IState) -> None:
        with self._lock:
            self._state = new_state

    def update(self, new_state : IState) -> None:
        with self._lock:
            self._state = new_state

    def get(self) -> IState:
        with self._lock:
            return self._state