import numpy as np
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from scipy.spatial.transform import Rotation as R, Slerp

def generate_trajectory_with_dt(
    start_pose: Pose,
    end_pose: Pose,
    n: int,
    avg_speed: float,
    start_yaw: float,
    final_yaw: float = float('nan'),
    yaw_fraction: float = 0.25  # proporción de la trayectoria donde se alcanza el yaw final
):
    trajectory = []

    s = np.linspace(0, 1, n)
    speed_profile = s**0.3
    speed_profile /= speed_profile.mean()
    speed_profile *= avg_speed

    start_pos = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
    end_pos = np.array([end_pose.position.x, end_pose.position.y, end_pose.position.z])
    direction = end_pos - start_pos
    distance_total = np.linalg.norm(direction)
    if distance_total == 0:
        raise ValueError("start_pose y end_pose no pueden ser iguales")
    direction_norm = direction / distance_total

    yaw0 = start_yaw
    if np.isnan(final_yaw):
        yawf = np.arctan2(direction[1], direction[0])
    else:
        yawf = final_yaw

    positions = []
    velocities = []

    for i in range(n):
        alpha = i / (n - 1)

        pos = start_pos * (1 - alpha) + end_pos * alpha
        positions.append(pos)

        # Ajuste del yaw: alcanza yawf en la fracción deseada de la trayectoria
        if alpha <= yaw_fraction:
            yaw_alpha = alpha / yaw_fraction  # mapea 0->yaw_fraction a 0->1
        else:
            yaw_alpha = 1.0
        yaw = (1 - yaw_alpha) * yaw0 + yaw_alpha * yawf
        quat = R.from_euler('z', yaw).as_quat()

        vel = speed_profile[i] * direction_norm
        velocities.append(vel)

        pose = Pose()
        pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
        pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        twist = Twist()
        twist.linear.x = vel[0]
        twist.linear.y = vel[1]
        twist.linear.z = vel[2]

        trajectory.append({'pose': pose, 'twist': twist, 'yaw': yaw})

    dt_list = []
    for i in range(n - 1):
        dist = np.linalg.norm(positions[i+1] - positions[i])
        vel_mag = np.linalg.norm(velocities[i])
        dt = dist / vel_mag if vel_mag > 0 else 0
        dt_list.append(dt)

    return trajectory, dt_list


import math

def leader_follower_offsets(num_vehicles: int, separation: float):
    """
    Genera offsets relativos para líder y followers.
    ENU primero, luego convierte a NED.
    - Para 2 drones: línea perpendicular a la dirección de avance (líder izquierda, follower derecha)
    - Para 3 drones: triángulo equilátero
    """
    offsets_enu = []

    if num_vehicles < 1:
        return []

    h = math.sqrt(3)/2 * separation  # altura del triángulo

    if num_vehicles == 1:
        offsets_enu.append((0.0, 0.0, 0.0))  # líder
    elif num_vehicles == 2:
        # Línea perpendicular: líder izquierda, follower derecha
        offsets_enu.append((-separation/2, 0.0, 0.0))  # líder izquierda
        offsets_enu.append((separation/2, 0.0, 0.0))   # follower derecha
    elif num_vehicles == 3:
        offsets_enu.append((0.0, 0.0, 0.0))           # líder punta
        offsets_enu.append((-separation/2, -h, 0.0))  # base izquierda
        offsets_enu.append((separation/2, -h, 0.0))   # base derecha
    else:
        raise NotImplementedError("Solo soportado hasta 3 drones")

    # Convertir ENU a NED
    offsets_ned = []
    for e, n, u in offsets_enu:
        x_ned = n      # Norte → X
        y_ned = e      # Este → Y
        z_ned = -u     # Up → -Z
        offsets_ned.append(np.array([x_ned, y_ned, z_ned]))

    return offsets_ned
