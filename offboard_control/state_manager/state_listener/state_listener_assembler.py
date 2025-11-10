from offboard_control.domain.state_listener.i_external_listener import IExternalListener
from offboard_control.domain.state_listener.i_external_listener_assembler import IExternalListenerAssembler
from offboard_control.domain.state_listener.i_external_listener import IExternalListener
from offboard_control.state_manager.state_listener.state_listener import StateListener
from offboard_control.state_manager.state_listener.mapper.external_mapper_factory import ExternalMapperFactory
from offboard_control.repository.state_repository import StateRepository
from offboard_control.state_manager.state_listener.event_source.event_source_factory import EventSourceFactory
from offboard_control.domain.model.configuration.communication_data_configuration import CommunicationDataConfiguration
from offboard_control.domain.model.configuration.i_protocol_configuration import IProtocolConfiguration

class StateListenerAssembler(IExternalListenerAssembler):

    def __init__(self):
        self.external_mapper_factory = ExternalMapperFactory()
        self.event_source_factory = EventSourceFactory()

    def assemble(
        self, 
        state_repository: StateRepository,
        communication_data_configuration: CommunicationDataConfiguration,
        protocol_configuration: IProtocolConfiguration
        ) -> IExternalListener:
        
        return StateListener(
            state_repository,
            self.external_mapper_factory.create(
                communication_data_configuration.external_type
            ),
            self.event_source_factory.create(
                communication_data_configuration,
                protocol_configuration
            )
        )
        
        
        