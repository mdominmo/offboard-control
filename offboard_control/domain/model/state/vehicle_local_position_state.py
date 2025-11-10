from offboard_control.domain.model.state.i_state import IState
from geometry_msgs.msg import Pose, Twist
from geographic_msgs.msg import GeoPose

class VehicleLocalPositionState(IState):
    def __init__(self):
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.ref_lat = 0.0
        self.ref_lon = 0.0
        self.ref_alt = 0.0

        self.heading = 0.0

    def get_pose(self):
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z
        return pose
    
    def get_twist(self):
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.linear.z = self.vz
        return twist
    
    def get_gps_ref(self):
        pose = GeoPose()
        pose.position.latitude = self.ref_lat
        pose.position.longitude = self.ref_lon
        pose.position.altitude = self.ref_alt
        return pose
