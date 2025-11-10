from abc import ABC, abstractmethod
from offboard_control.domain.model.configuration.i_configuration import IConfiguration

class VehicleConfiguration(IConfiguration):
    def __init__(
            self,
            vehicle_id: int,
            autopilot_id: str,
            vehicle_type: str
            ):
        self.vehicle_id = vehicle_id
        self.autopilot_id = autopilot_id
        self.vehicle_type = vehicle_type
