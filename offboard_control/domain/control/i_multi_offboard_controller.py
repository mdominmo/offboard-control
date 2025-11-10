from abc import ABC, abstractmethod
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPose

class IMultiOffboardController(ABC):

    @abstractmethod
    def arm(self, ids: list[int]):
        pass

    @abstractmethod
    def arm_all(self):
        pass

    @abstractmethod
    def disarm_all(self):
        pass

    @abstractmethod
    def disarm(self, ids: list[int]):
        pass

    @abstractmethod
    def take_off(self, ids: list[int], height: float):
        pass

    @abstractmethod
    def take_off_all(self, hegiht: float):
        pass

    @abstractmethod
    def hold(self, ids: list[int]):
        pass

    @abstractmethod
    def hold_all(self):
        pass

    @abstractmethod
    def return_to_launch(self, ids: list[int]):
        pass

    @abstractmethod
    def return_to_launch_all(self):
        pass

    @abstractmethod
    def land(self, ids: list[int]):
        pass

    @abstractmethod
    def land_all(self):
        pass

    @abstractmethod
    def go_to(self, id: int, pose: Pose):
        pass

    @abstractmethod
    def go_to_all(self, poses: list[GeoPose]):
        pass

    @abstractmethod
    def go_to_local(self, ids: list[int], gps_origin: GeoPose, poses: list[Pose]):
        pass

    @abstractmethod
    def go_to_all_local(self, gps_origin: GeoPose, poses: list[Pose]):
        pass
    
    @abstractmethod
    def local_waypoint_following_all(
            self,
            gps_origin: GeoPose,
            waypoints: list[list[Pose]]
        ):
        pass
    
    @abstractmethod
    def trajectory_following(
            self, 
            ids: list[int], 
            gps_origin: GeoPose, 
            trajectories,
            dt=1.0
        ):
        pass

    @abstractmethod
    def trajectory_following_all(
            self, 
            gps_origin: GeoPose, 
            trajectories,
            dt = 1.0
        ):
        pass

    @abstractmethod
    def cancel_operation_all(self):
        pass
    
    @abstractmethod
    def cancel_operation(self, ids: list[int]):
        pass
    