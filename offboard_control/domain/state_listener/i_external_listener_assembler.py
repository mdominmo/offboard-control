from abc import ABC, abstractmethod
from offboard_control.domain.state_listener.i_external_listener import IExternalListener
from offboard_control.domain.model.configuration.communication_configuration import CommunicationConfiguration
from offboard_control.repository.state_repository import StateRepository

class IExternalListenerAssembler(ABC):

    @abstractmethod
    def assemble(
        self, 
        state_repository: StateRepository,
        external_state: type,
        communication_configuration: CommunicationConfiguration
        ) -> IExternalListener:
        pass

        
       