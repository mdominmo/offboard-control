from offboard_control.domain.state_manager.i_state_manager import IStateManager
from offboard_control.state_manager.state_manager import StateManager
from offboard_control.domain.model.configuration.communication_configuration import CommunicationConfiguration
from offboard_control.domain.state_manager.i_state_manager_assembler import IStateManagerAssembler
from offboard_control.state_manager.state_listener.state_listener_assembler import StateListenerAssembler
from offboard_control.repository.state_repository_factory import StateRepositoryFactory
from offboard_control.domain.constant.states import States

class StateManagerAssembler(IStateManagerAssembler):
    def __init__(self):
        self.state_repository_factory = StateRepositoryFactory()
        self.state_listener_assembler = StateListenerAssembler()
    
    def assemble(self, configuration: CommunicationConfiguration) -> IStateManager:
        
        state_listeners = []
        state_repositories = {}

        state_repositories[States.OPERATION_STATUS] = self.state_repository_factory.create(States.OPERATION_STATUS)

        for internal_state, data_configuration in configuration.communication_data_configuration.items():
            
            state_repository = self.state_repository_factory.create(internal_state)
            state_repositories[internal_state] = state_repository

            state_listeners.append(
                self.state_listener_assembler.assemble(
                    state_repository,
                    data_configuration,
                    configuration.protocol_configuration
                ))
            
        return StateManager(state_listeners, state_repositories)