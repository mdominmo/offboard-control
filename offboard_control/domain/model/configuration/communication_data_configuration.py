from abc import ABC, abstractmethod
from offboard_control.domain.model.configuration.i_configuration import IConfiguration

class CommunicationDataConfiguration(IConfiguration):
    
    def __init__(
            self, 
            internal_id: str,
            external_id: str,
            internal_type,
            external_type
            ):
        self.internal_id = internal_id
        self.external_id = external_id
        self.internal_type = internal_type
        self.external_type = external_type