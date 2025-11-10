from enum import Enum

class OutgoingCommunication(Enum):
    OFFBOARD_MODE = "OFFBOARD_MODE"
    TRAJECTORY = "TRAJECTORY"
    WAYPOINT = "WAYPOINT"
    COMMAND = "COMMAND"

