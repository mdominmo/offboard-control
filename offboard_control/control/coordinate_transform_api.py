from geographic_msgs.msg import GeoPose
from geographiclib.geodesic import Geodesic
import numpy as np
import math
from geometry_msgs.msg import Pose

def gps_to_local(self, lat1, lon1, alt1, lat2, lon2, alt2):
        geod = Geodesic.WGS84
        g = geod.Inverse(lat1, lon1, lat2, lon2)

        north = g['s12'] * math.cos(math.radians(g['azi1']))
        east  = g['s12'] * math.sin(math.radians(g['azi1']))
        down  = alt1 - alt2

        return north, east, down
    
def gps_to_ned(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    g = Geodesic.WGS84.Inverse(lat_ref, lon_ref, lat, lon)
    distance = g['s12']
    azimuth = np.radians(g['azi1'])
    north = distance * np.cos(azimuth)
    east = distance * np.sin(azimuth)
    down = alt_ref - alt 
    return north, east, down

def get_local_ned_target(drone_gps, map_origin_gps: GeoPose, target_global_ned: Pose) -> Pose:
    
    # posición del dron en el marco NED global
    drone_north, drone_east, drone_down = gps_to_ned(
        drone_gps.lat, drone_gps.lon, drone_gps.alt,
        map_origin_gps.position.latitude,
        map_origin_gps.position.longitude,
        map_origin_gps.position.altitude
    )

    # extraer objetivo global en NED desde el Pose
    target_north = target_global_ned.position.x
    target_east = target_global_ned.position.y
    target_down = target_global_ned.position.z

    # diferencia entre el objetivo y la posición actual del dron
    delta_north = target_north - drone_north
    delta_east = target_east - drone_east
    delta_down = target_down - drone_down

    # construir nuevo Pose con coordenadas relativas (locales)
    relative_pose = Pose()
    relative_pose.position.x = delta_north
    relative_pose.position.y = delta_east
    relative_pose.position.z = delta_down
    relative_pose.orientation = target_global_ned.orientation  # copiar orientación si te interesa

    return relative_pose