from px4_msgs.msg import VehicleStatus # type: ignore
from px4_msgs.msg import VehicleCommand # type: ignore
from px4_msgs.msg import OffboardControlMode # type: ignore
from px4_msgs.msg import TrajectorySetpoint # type: ignore
from px4_msgs.msg import VehicleGlobalPosition 
from px4_msgs.msg import VehicleLocalPosition 
from rclpy.node import Node
from offboard_control.domain.service.i_configuration_service import IConfigurationService
from offboard_control.domain.model.configuration.vehicle_configuration import VehicleConfiguration
from offboard_control.domain.model.configuration.ros2_protocol_configuration import Ros2ProtocolConfiguration
from offboard_control.domain.model.configuration.communication_configuration import CommunicationConfiguration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from offboard_control.domain.model.configuration.vehicle_configuration import VehicleConfiguration
from offboard_control.domain.model.configuration.offboard_configuration import OffboardConfiguration
from offboard_control.domain.constant.communication_direction import CommunicationDirection
from offboard_control.domain.constant.states import States
from offboard_control.domain.model.configuration.ros2_communication_data_configuration import Ros2CommunicationDataConfiguration
from offboard_control.domain.constant.outgoing_communication import OutgoingCommunication
from geographic_msgs.msg import GeoPose
import rclpy

class RealUAVSConfigurationService(IConfigurationService):
    
    def __init__(
            self, 
            node: Node,
            vehicle_ids
        ):
        self.node = node
        self.vehicle_ids = vehicle_ids

    def get_offboard_configurations(self) -> OffboardConfiguration:
        
        configurations = []

        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )

        for rc_id in self.vehicle_ids:
            
            drone_namespace = f"px4_{str(rc_id)}"

            vehicle_configuration = VehicleConfiguration(
                vehicle_id=rc_id,
                autopilot_id="PX4",
                vehicle_type="multirotor"
            )

            ros2_protocol_configuration = Ros2ProtocolConfiguration(
                qos_profile=qos_profile,
                node=self.node
            )

            incomming_data_configuration = {}

            # Ensure that we can reach these messages
            incomming_data_configuration[States.STATUS] = Ros2CommunicationDataConfiguration(
                internal_id = States.STATUS,
                external_id = "VehicleStatus",
                external_type = VehicleStatus,
                topic = f'/{drone_namespace}/fmu/out/vehicle_status',
                msg_type = VehicleStatus
            )

            incomming_data_configuration[States.GLOBAL_POSITION] = Ros2CommunicationDataConfiguration(
                internal_id = States.GLOBAL_POSITION,
                external_id = "VehicleGlobalPosition",
                external_type = VehicleGlobalPosition,
                topic = f'/{drone_namespace}/fmu/out/vehicle_global_position',
                msg_type = VehicleGlobalPosition
            )

            incomming_data_configuration[States.LOCAL_POSITION] = Ros2CommunicationDataConfiguration(
                internal_id = States.LOCAL_POSITION,
                external_id = "VehicleLocalPosition",
                external_type = VehicleLocalPosition,
                topic = f'/{drone_namespace}/fmu/out/vehicle_local_position',
                msg_type = VehicleLocalPosition
            )

            incomming_communication_configuration = CommunicationConfiguration(
                CommunicationDirection.INCOMING,
                incomming_data_configuration,
                ros2_protocol_configuration
            )

            outgoing_data_configuration = {}

            outgoing_data_configuration[OutgoingCommunication.COMMAND] = Ros2CommunicationDataConfiguration(
                external_id = OutgoingCommunication.COMMAND,
                external_type = VehicleCommand,
                topic = f'/{drone_namespace}/fmu/in/vehicle_command',
                msg_type = VehicleCommand
            )

            outgoing_data_configuration[OutgoingCommunication.OFFBOARD_MODE] = Ros2CommunicationDataConfiguration(
                external_id = OutgoingCommunication.OFFBOARD_MODE,
                external_type = OffboardControlMode,
                topic = f"/{drone_namespace}/fmu/in/offboard_control_mode",
                msg_type = OffboardControlMode
            )

            outgoing_data_configuration[OutgoingCommunication.TRAJECTORY] = Ros2CommunicationDataConfiguration(
                external_id = OutgoingCommunication.TRAJECTORY,
                external_type = TrajectorySetpoint,
                topic = f'/{drone_namespace}/fmu/in/trajectory_setpoint',
                msg_type = TrajectorySetpoint
            )

            outgoing_communication_configuration = CommunicationConfiguration(
                CommunicationDirection.OUTGOING,
                outgoing_data_configuration,
                ros2_protocol_configuration
            )
                
            configurations.append(
                OffboardConfiguration(
                    incomming_communication_configuration,
                    outgoing_communication_configuration,
                    vehicle_configuration)
                )
            
        return configurations
    

    #TODO
    # Individual config retrieval
    def get_offboard_configuration(self) -> OffboardConfiguration:
        pass
       
    
    def get_auto_gps_reference(self, vehicle_id):

        self.node.get_logger().info(f"[Offboard_configuration_service: Configuring auto GPS reference...")

        if not vehicle_id in self.vehicle_ids:
            raise ValueError(f"Te vehicle id {vehicle_id} is not in the fleet.")
        
        gps_origin = GeoPose()
        def callback(msg: VehicleLocalPosition):

            if msg.xy_global and msg.z_global:
                gps_origin.position.latitude = msg.ref_lat
                gps_origin.position.longitude = msg.ref_lon
                gps_origin.position.altitude = msg.ref_alt

        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
        
        sub = self.node.create_subscription(
            VehicleLocalPosition,
            f'/px4_{vehicle_id}/fmu/out/vehicle_local_position',
            callback, qos_profile
        )

        while 0.0 in (gps_origin.position.latitude, gps_origin.position.longitude, gps_origin.position.altitude): 
            rclpy.spin_once(self.node, timeout_sec=5.0)
            self.node.get_logger().info(f"[Offboard_configuration_service: Waiting for GPS reference from uav: {vehicle_id}]")
        
        self.node.get_logger().info(f"[Offboard_configuration_service: GPS origin settled in lat: {gps_origin.position.latitude}, lon: {gps_origin.position.longitude}, alt: {gps_origin.position.altitude} from uav: {vehicle_id}]")

        self.node.destroy_subscription(sub)
        return gps_origin