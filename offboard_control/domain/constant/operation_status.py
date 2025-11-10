from enum import Enum

class OperationStatus(Enum):
    RUNNING = 0
    FINISHED = 1
    SUSPENDED = 2
    RESUMED = 3
    CANCELED = 4
    