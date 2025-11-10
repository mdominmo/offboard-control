from abc import ABC, abstractmethod
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPose

class ISingleOffboardController(ABC):
    
    @abstractmethod
    def arm(self) -> None:
        pass

    @abstractmethod
    def disarm(self) -> None:
        pass

    @abstractmethod
    def take_off(self) -> None:
        pass

    # @abstractmethod
    # def offboard(self) -> None:
    #     pass

    @abstractmethod
    def go_to(self, pose: GeoPose) -> None:
        pass

    @abstractmethod
    def return_to_launch(self) -> None:
        pass

    @abstractmethod
    def land(self) -> None:
        pass

    @abstractmethod
    def cancel_operation(self) -> None:
        pass

    @abstractmethod
    def trajectory_following(self, trajectory: list[Pose]) -> None:
        pass

    
    
