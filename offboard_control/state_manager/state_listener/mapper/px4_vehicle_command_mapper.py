from px4_msgs.msg import VehicleCommand # type: ignore
from offboard_control.domain.mapper.i_external_mapper import IExternalMapper
from offboard_control.domain.model.i_domain import IDomain

class Px4VehicleCommandMapper(IExternalMapper):

    def map_to(self, domain: IDomain) -> VehicleCommand: 
        pass
        #TODO
        #return VehicleCommand()

    def map_from(self, autopilot_state) -> IDomain:
        pass
        
       