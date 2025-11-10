from offboard_control.domain.state_listener.i_external_listener import IExternalListener
from offboard_control.domain.mapper.i_external_mapper import IExternalMapper
from offboard_control.domain.state_listener.i_event_source import IEventSource
from offboard_control.domain.repository.i_repository import IRepository

class StateListener(IExternalListener):

    def __init__(
            self, 
            repository: IRepository,
            mapper: IExternalMapper,
            event_source: IEventSource
        ):
        
        self.repository = repository
        self.mapper = mapper
        self.event_source = event_source

        self.event_source.subscribe(self.event_handler)

    def event_handler(self, external) -> None:
        self.repository.update(
            self.mapper.map_from(external)
        )