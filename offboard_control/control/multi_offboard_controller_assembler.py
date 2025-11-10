from rclpy.node import Node
from offboard_control.domain.control.i_multi_offboard_controller_assembler import IMultiOffboardControllerAssembler
from offboard_control.domain.control.i_multi_offboard_controller import IMultiOffboardController
from offboard_control.control.single_offboard_controller_assembler import SingleOffboardControllerAssembler
from offboard_control.domain.model.configuration.offboard_configuration import OffboardConfiguration
from offboard_control.control.multi_offboard_controller import MultiOffboardController
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class MultiOffboardControllerAssembler(IMultiOffboardControllerAssembler):

    def __init__(self):
        self.offboard_controller_assembler = SingleOffboardControllerAssembler()

    def assemble(
            self, 
            node: Node,
            offboard_configuration: list[OffboardConfiguration],
            num_threads = 4
            ) -> IMultiOffboardController:
        
        self.executor = MultiThreadedExecutor(num_threads = num_threads)
        self.executor.add_node(node)
        operation_group = ReentrantCallbackGroup()

        controllers = []
        for configuration in offboard_configuration:

            controllers.append(
                self.offboard_controller_assembler.assemble(
                    node,
                    configuration,
                    operation_group
                ))
            
        return MultiOffboardController(
            node,
            controllers
        )


       

    