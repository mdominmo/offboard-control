from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from offboard_control.domain.model.configuration.i_protocol_configuration import IProtocolConfiguration

class Ros2ProtocolConfiguration(IProtocolConfiguration):
    def __init__(
            self,
            qos_profile: QoSProfile,
            node: Node
        ):

        self.node = node
        self.qos_profile = qos_profile

        if qos_profile is None:
            self.qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )


        
        
        
        
