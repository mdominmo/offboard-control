from abc import ABC, abstractmethod
from rclpy.node import Node
from offboard_control.domain.model.configuration.communication_configuration import CommunicationConfiguration
from offboard_control.domain.control.i_single_offboard_controller import ISingleOffboardController

class ISingleOffboardControllerAssembler(ABC):

    def assemble(
            self, 
            node: Node,
            system_id: int,
            autopilot_id: str,
            communication_configuration: list[CommunicationConfiguration]
        ) -> ISingleOffboardController:
        pass    