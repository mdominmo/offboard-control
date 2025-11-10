from abc import ABC, abstractmethod
from rclpy.node import Node
from offboard_control.domain.control.i_multi_offboard_controller import IMultiOffboardController
from offboard_control.domain.model.configuration.offboard_configuration import OffboardConfiguration

class IMultiOffboardControllerAssembler(ABC):

    @abstractmethod
    def assemble(
            self, 
            node: Node,
            offboard_configuration: list[OffboardConfiguration]
        ) -> IMultiOffboardController:
        pass