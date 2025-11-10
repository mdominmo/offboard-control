from rclpy.node import Node
from offboard_control.dispatcher.command_dispatcher_factory import CommandDispatcherFactory
from offboard_control.state_manager.state_manager_assembler import StateManagerAssembler
from offboard_control.domain.model.configuration.communication_configuration import CommunicationConfiguration
from offboard_control.control.offboard_controller import OffboardController
from offboard_control.domain.control.i_single_offboard_controller_assembler import ISingleOffboardControllerAssembler
from offboard_control.domain.model.configuration.offboard_configuration import OffboardConfiguration
from rclpy.callback_groups import ReentrantCallbackGroup

class SingleOffboardControllerAssembler(ISingleOffboardControllerAssembler):

    def __init__(self):

        self.command_dispatcher_factory = CommandDispatcherFactory()
        self.state_manager_assembler = StateManagerAssembler()
          
    def assemble(
            self, 
            node: Node,
            configuration: OffboardConfiguration,
            operation_group: ReentrantCallbackGroup
        ) -> OffboardController:

        return OffboardController(
            node,
            self.command_dispatcher_factory.create(
                configuration.vehicle_configuration.vehicle_id,
                configuration.vehicle_configuration.autopilot_id, 
                configuration.outgoing_communication_configuration
            ),
            self.state_manager_assembler.assemble(configuration.incomming_communication_configuration),
            operation_group
        )