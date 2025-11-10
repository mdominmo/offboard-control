from offboard_control.domain.model.configuration.communication_data_configuration import CommunicationDataConfiguration

class Ros2CommunicationDataConfiguration(CommunicationDataConfiguration):
    def __init__(
            self, 
            internal_id: str = None,
            external_id: str = None,
            internal_type: type = None,
            external_type: type = None,
            topic: str = None,
            msg_type: type = None
        ):
        super().__init__(
            internal_id,
            external_id, 
            internal_type, 
            external_type
        )
        self.topic = topic
        self.msg_type = msg_type

        
        
        
        
