from rclpy.clock import Clock
from px4_msgs.msg import OffboardControlMode # type: ignore
from px4_msgs.msg import TrajectorySetpoint # type: ignore
from px4_msgs.msg import VehicleCommand # type: ignore
from offboard_control.domain.dispatcher.i_command_dispatcher import ICommandDispatcher
from offboard_control.domain.model.configuration.communication_configuration import CommunicationConfiguration
from offboard_control.domain.constant.outgoing_communication import OutgoingCommunication
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPose

class Px4CommandDispatcher(ICommandDispatcher):

    def __init__(
            self, 
            vehicle_id: int,
            communication_configuration: CommunicationConfiguration
        ):

        self.vehicle_id = vehicle_id
        self.communication_configuration = communication_configuration
        self.protocol_configuration = communication_configuration.protocol_configuration
        self.communication_data_configuration = communication_configuration.communication_data_configuration
        self.node = self.protocol_configuration.node

        self.publisher_offboard_mode = self.protocol_configuration.node.create_publisher(
            self.communication_data_configuration[OutgoingCommunication.OFFBOARD_MODE].msg_type,
            self.communication_data_configuration[OutgoingCommunication.OFFBOARD_MODE].topic,
            self.protocol_configuration.qos_profile
        )

        self.publisher_trajectory = self.protocol_configuration.node.create_publisher(
            self.communication_data_configuration[OutgoingCommunication.TRAJECTORY].msg_type,
            self.communication_data_configuration[OutgoingCommunication.TRAJECTORY].topic,
            self.protocol_configuration.qos_profile
        )

        self.command_publisher = self.protocol_configuration.node.create_publisher(
            self.communication_data_configuration[OutgoingCommunication.COMMAND].msg_type,
            self.communication_data_configuration[OutgoingCommunication.COMMAND].topic,
            self.protocol_configuration.qos_profile
        )

    def arm(self) -> None:
        self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.node.get_logger().debug("Px4 Arm command sent")

    def disarm(self) -> None:
        self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.node.get_logger().debug("Px4 Disarm command sent")

    def take_off(self, height=10.0) -> None:
        # self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param2 = 3.0, param7 = height)
        self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,param1=1.0,param7=height)
        self.node.get_logger().debug("Px4 Takeoff command sent")

    def offboard(self) -> None:
        self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.node.get_logger().debug("Px4 Offboard command sent")

    def hold(self) -> None:
        self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 4., 3.)
        self.node.get_logger().debug("Px4 Hold command sent")

    def set_home(self, home: GeoPose):
        self.publish_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_HOME, 
            param1=0.,
            param5=home.position.latitude,
            param6=home.position.longitude,
            param7=home.position.altitude
        )
        self.node.get_logger().debug("Px4 Set Home sent")

    def go_to(
            self, 
            pose: Pose,
            velocity = None,
            yaw = 0.0,
            yaw_speed = float('nan')
        ) -> None:         

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        if not velocity:
            trajectory_msg.velocity[0] = float('nan')
            trajectory_msg.velocity[1] = float('nan')
            trajectory_msg.velocity[2] = float('nan')

        else:
            trajectory_msg.velocity[0] = velocity.linear.x
            trajectory_msg.velocity[1] = velocity.linear.y
            trajectory_msg.velocity[2] = velocity.linear.z

        trajectory_msg.position[0] = pose.position.x
        trajectory_msg.position[1] = pose.position.y
        trajectory_msg.position[2] = pose.position.z
        
        trajectory_msg.acceleration[0] = float('nan')
        trajectory_msg.acceleration[1] = float('nan')
        trajectory_msg.acceleration[2] = float('nan')

        trajectory_msg.yaw = yaw
        self.node.get_logger().debug(f"sending yaw {yaw} to PX4")
        trajectory_msg.yawspeed = yaw_speed
        
        self.node.get_logger().debug(f"drone_{self.vehicle_id} x: {pose.position.x}, y: {pose.position.y}, yaw: {trajectory_msg.yaw}")
        self.node.get_logger().debug("Dispatcher pose: {pose}")

        self.publisher_trajectory.publish(trajectory_msg)

    def return_to_launch(self) -> None:
        self.publish_command(
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.node.get_logger().debug("Return to launch command sent")

    def land(self) -> None:
        self.publish_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.node.get_logger().debug("Land command sent")

    def offboard_heartbeat(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)          

    def publish_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=float('nan'), param5=float('nan'), param6=float('nan'), param7=10.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = (1 + self.vehicle_id)  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.command_publisher.publish(msg)
