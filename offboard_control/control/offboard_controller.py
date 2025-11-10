from rclpy.node import Node
from rclpy.task import Future
from offboard_control.domain.control.i_single_offboard_controller import ISingleOffboardController
from offboard_control.domain.constant.vehicle_status_states import VehicleStatusStates
from offboard_control.domain.dispatcher.i_command_dispatcher import ICommandDispatcher
from offboard_control.domain.state_manager.i_state_manager import IStateManager
from offboard_control.domain.constant.states import States
from offboard_control.domain.model.state.operation_status_state import OperationStatusState
from offboard_control.domain.constant.operation_status import OperationStatus
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import math
from offboard_control.control.coordinate_transform_api import gps_to_local, gps_to_ned
from rclpy.clock import Clock
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
from offboard_control.control import utils


class OffboardController(ISingleOffboardController):

    def __init__(
            self,
            node: Node,
            command_dispatcher: ICommandDispatcher,
            state_manager: IStateManager,
            operation_group: ReentrantCallbackGroup
        ):
        self.node = node
        self.command_dispatcher = command_dispatcher
        self.state_manager = state_manager
        self.command_period = 0.02
        self.operation_timer = None
        self.heartbeat_timer = None
        self.operation_future = None
        self.operation_group = operation_group
        self.clock = Clock()

        self.is_external_operation_active = False
        self._target_pose = None
        self._target_velocity = None
        self._target_yaw = None

        self.node.create_timer(
            self.command_period,
            self.heartbeat_callback,
            operation_group
        )


    def set_external_target(self, pose: Pose, velocity: Twist, yaw: float):
        self._target_pose = pose
        self._target_velocity = velocity
        self._target_yaw = yaw

    
    def heartbeat_callback(self):
        self.offboard_heartbeat()


    def arm(self) -> None:
        
        self.future = Future()

        if self.check_failsafe():
            self.future.set_result({
                "success": False,
                "msg": "Arm operation failed"
            })
            return self.future

        def callback():
            
            if self.check_failsafe():
                self.operation_timer.cancel()
                self.future.set_result({
                    "success": False,
                    "msg": "Arm operation failed"
                })
                return self.future
        
            if self.check_arm():
                self.operation_timer.cancel()
                self.node.get_logger().debug(f"Vehicle Armed")
                self.future.set_result({
                    "success": True,
                    "msg": "Vehicle Armed"
                })
                return
            
            self.command_dispatcher.arm()

        self.operation_timer = self.node.create_timer(self.command_period, callback, callback_group=self.operation_group)
        return self.future
    
    
    def disarm(self) -> None:

        self.future = Future()

        if self.check_failsafe():
            self.future.set_result({
                "success": False,
                "msg": "Disarm operation failed"
            })
            return self.future

        def callback():

            if self.check_failsafe():
                self.operation_timer.cancel()
                self.future.set_result({
                    "success": False,
                    "msg": "Disarm operation failed"
                })
                return self.future
            
            if not self.check_arm():
                self.operation_timer.cancel()
                self.node.get_logger().debug(f"Vehicle disarmed")
                self.future.set_result({
                "success": True,
                "msg": "Vehicle disarmed"
            })
                return
            self.command_dispatcher.disarm()

        self.operation_timer = self.node.create_timer(self.command_period, callback, callback_group=self.operation_group)
        return self.future


    # def take_off(self, height=10.0) -> None:

    #     self.future = Future()
    #     tol = 1.5

    #     ref_amsl_height = self.state_manager.state_repositories[States.GLOBAL_POSITION].get().alt
    #     z0 = self.state_manager.state_repositories[States.LOCAL_POSITION].get().z
        
    #     self.command_dispatcher.arm()
    #     self.command_dispatcher.take_off(height + ref_amsl_height)
        
    #     def callback():
  
    #         if self.check_failsafe():
    #             self.operation_timer.cancel()
    #             self.future.set_result({
    #                 "success": False,
    #                 "msg": "Takeoff operation failed"
    #             })
    #             return self.future

    #         current_height = self.state_manager.state_repositories[States.LOCAL_POSITION].get().z

    #         if self.check_hold() and abs((-current_height + z0) - height) < tol :
    #             self.operation_timer.cancel()
    #             self.future.set_result({
    #             "success": True,
    #             "msg": "Vehicle on air"
    #         })
    #             return
    #         self.command_dispatcher.arm()

    #     self.operation_timer = self.node.create_timer(self.command_period, callback, callback_group=self.operation_group)
    #     return self.future
    

    def take_off(
        self, 
        height=10.0,
        n_points = 150
    ) -> None:

        ref = self.state_manager.state_repositories[States.LOCAL_POSITION].get().get_gps_ref()
        yaw = self.state_manager.state_repositories[States.LOCAL_POSITION].get().heading
        p0 = self.state_manager.state_repositories[States.LOCAL_POSITION].get().get_pose()
        p0.position.z-=height
        
        return self.go_to_local(ref, p0, n_points=n_points, final_yaw=yaw)    
    
    
    def hold(self) -> None:
        self.command_dispatcher.hold()
    
    
    def go_to(self, pose: GeoPose) -> None:

        self.future = Future()

        if self.check_failsafe():
            self.future.set_result({
                "success": False,
                "msg": "Goto operation failed"
            })
            return self.future

        global_target_position = pose.position
        reached_counter = 0

        if not self.check_arm():
            self.node.get_logger().debug(f"Waypoint following failed")
            self.future.set_result({
                "success": False,
                "msg": "Waypoint following failed"
            })
            return self.future

        def callback():

            if self.check_failsafe():
                self.operation_timer.cancel()
                self.future.set_result({
                    "success": False,
                    "msg": "Goto operation failed"
                })
                return self.future

            current_global_position = self.state_manager.state_repositories[States.GLOBAL_POSITION].get()
            current_local_position = self.state_manager.state_repositories[States.LOCAL_POSITION].get()

            d_north, d_east, d_down = gps_to_local(
                current_global_position.lat, current_global_position.lon, current_global_position.alt,
                global_target_position.latitude, global_target_position.longitude, global_target_position.altitude
            )

            local_target_pose = Pose()
            local_target_pose.position.x = current_local_position.x + d_north
            local_target_pose.position.y = current_local_position.y + d_east
            local_target_pose.position.z = current_local_position.z + d_down

            self.command_dispatcher.go_to(local_target_pose)

            nonlocal reached_counter

            if all(abs(c - t) < 2 for c, t in zip((
                current_local_position.x, 
                current_local_position.y, 
                current_local_position.z), 
                (local_target_pose.position.x, 
                 local_target_pose.position.y, 
                 local_target_pose.position.z)
                 )):
                reached_counter+=1
                if reached_counter == 20:
                    self.operation_timer.cancel()
                    self.node.get_logger().debug(f"Position reached")
                    self.future.set_result({
                        "success": True,
                        "msg": "Position reached"
                    })
                    return

        self.operation_timer = self.node.create_timer(self.command_period, callback, callback_group=self.operation_group)
        return self.future
    

    def go_to_local(
            self,
            gps_origin: GeoPose,
            pose: Pose,
            n_points = 80,
            speed = 2.0,
            final_yaw = float('nan'),
            yaw_fraction = 0.25
        ) -> None:

        self.future = Future()
        reached_counter = 0
        target_local_pose = Pose()

        # TODO Check the orientation
        current_pose = self.state_manager.state_repositories[States.LOCAL_POSITION].get()

        o_lat = current_pose.ref_lat
        o_lon = current_pose.ref_lon
        o_alt = current_pose.ref_alt

        d_origin_x, d_origin_y, d_origin_z = gps_to_ned(
                o_lat, 
                o_lon, 
                o_alt,
                gps_origin.position.latitude,
                gps_origin.position.longitude,
                gps_origin.position.altitude
        )

        start_pose  = Pose()
        start_pose.position.x = current_pose.x
        start_pose.position.y = current_pose.y   
        start_pose.position.z = current_pose.z   
        start_yaw = current_pose.heading

        target_local_pose.position.x = pose.position.x - d_origin_x
        target_local_pose.position.y = pose.position.y - d_origin_y
        target_local_pose.position.z = pose.position.z - d_origin_z
        target_local_pose.orientation = pose.orientation 

        trajectory, dts = utils.generate_trajectory_with_dt(
            start_pose, 
            target_local_pose,
            n_points,
            speed,
            start_yaw,
            final_yaw,
            yaw_fraction
        )

        start_time = self.clock.now()
        
        def callback():

            nonlocal reached_counter, target_local_pose, start_time, trajectory
            
            if not self.check_offboard():
                self.operation_timer.cancel()
                self.future.set_result({
                    "success": True,
                    "msg": "Not in offboard, Returning control to the pilot"
                })
                return self.future
            
            elapsed_time = (self.clock.now() - start_time).nanoseconds / 1e9
            time_from_start = [0.0] + np.cumsum(dts).tolist()
            if elapsed_time >= time_from_start[-1]:
                # Fin de trayectoria
                self.future.set_result({
                    "success": True,
                    "msg": "Trajectory completed"
                })
                self.node.get_logger().debug("Trajectory finished")
                self.operation_timer.cancel()
                return

            index = max([i for i, t in enumerate(time_from_start) if t <= elapsed_time])
            
            target_local_pose = Pose()
            target_local_pose.position.x = trajectory[index]['pose'].position.x - d_origin_x
            target_local_pose.position.y = trajectory[index]['pose'].position.y - d_origin_y
            target_local_pose.position.z = trajectory[index]['pose'].position.z - d_origin_z
     
            target_yaw = float(trajectory[index]['yaw'])
            target_velocity = trajectory[index]["twist"]

            self.command_dispatcher.go_to(
                target_local_pose,
                # target_velocity,
                yaw=target_yaw
            )  

        self.operation_timer = self.node.create_timer(self.command_period, callback, callback_group=self.operation_group)
        return self.future
    

    def local_waypoint_following(
            self,
            gps_origin: GeoPose, 
            waypoints: list[Pose],
            headings: list[float] = None
            ):
        
        waypoint_index = 0
        reach_counter = 0
        self.future = Future()
        setpoint_sent = False
        target_local_pose = Pose()
        
        self.node.get_logger().debug(f"Vehicle following waypoint")

        def auto_heading(current_wp, next_wp):
            
            self.node.get_logger().debug(f"Current wp: {[current_wp.x, current_wp.y, current_wp.z]}, Next wp: {next_wp}")
            dx = next_wp.x - current_wp.x
            dy = next_wp.y - current_wp.y 

            yaw = math.atan2(dy, dx)
            self.node.get_logger().debug(f"yaw autoheading {yaw}")
            return yaw
        
        def callback():
            
            nonlocal waypoints, waypoint_index, reach_counter, setpoint_sent

            if not self.check_offboard():
                self.operation_timer.cancel()
                self.future.set_result({
                    "success": True,
                    "msg": "Not in offboard, Returning control to the pilot"
                })
                return self.future
            
            if not setpoint_sent:

                local_position = self.state_manager.state_repositories[States.LOCAL_POSITION].get()
                o_lat = local_position.ref_lat
                o_lon = local_position.ref_lon
                o_alt = local_position.ref_alt

                d_origin_x, d_origin_y, d_origin_z = gps_to_ned(
                        o_lat, 
                        o_lon, 
                        o_alt,
                        gps_origin.position.latitude,
                        gps_origin.position.longitude,
                        gps_origin.position.altitude
                )
                
                target_local_pose.position.x = waypoints[waypoint_index].position.x - d_origin_x
                target_local_pose.position.y = waypoints[waypoint_index].position.y - d_origin_y
                target_local_pose.position.z = waypoints[waypoint_index].position.z - d_origin_z
                target_local_pose.orientation = waypoints[waypoint_index].orientation 

                if headings is None:
                    target_heading = auto_heading(local_position, target_local_pose.position)
                else:
                    target_heading = headings[waypoint_index]
                
                self.node.get_logger().debug(f"target_heading: {target_heading}")
                self.command_dispatcher.go_to(
                    target_local_pose,
                    yaw = target_heading,
                    yaw_speed=0.2
                )
                setpoint_sent = True
            
            current_pose = self.state_manager.state_repositories[States.LOCAL_POSITION].get()
            if all(abs(c - t) < 2 for c, t in zip(
                (current_pose.x, current_pose.y, current_pose.z),
                (target_local_pose.position.x, target_local_pose.position.y, target_local_pose.position.z)
            )):
                reach_counter += 1
                if reach_counter == 10:
                    waypoint_index += 1
                    self.node.get_logger().info(f"waypoint {waypoint_index} reached")
                    reach_counter = 0
                    setpoint_sent=False

            if waypoint_index >= len(waypoints):
                self.future.set_result({
                    "success": True,
                    "msg": "waypoint following completed"
                })
                self.node.get_logger().debug(f"waypoint following finished")
                self.operation_timer.cancel()
                return 
            
        self.operation_timer = self.node.create_timer(self.command_period, callback, callback_group=self.operation_group)
        return self.future
    

    def trajectory_following(
        self,
        gps_origin: GeoPose, 
        poses: list[Pose],
        velocities: list[Twist],
        yaw: list[float],
        dts: list[float]
    ) -> None:

        self.future = Future()
        start_time = self.clock.now()

        local_position = self.state_manager.state_repositories[States.LOCAL_POSITION].get()
        o_lat = local_position.ref_lat
        o_lon = local_position.ref_lon
        o_alt = local_position.ref_alt

        d_origin_x, d_origin_y, d_origin_z = gps_to_ned(
            o_lat, 
            o_lon, 
            o_alt,
            gps_origin.position.latitude,
            gps_origin.position.longitude,
            gps_origin.position.altitude
        )

        def callback():
            nonlocal start_time

            if not self.check_offboard():
                self.operation_timer.cancel()
                self.future.set_result({
                    "success": True,
                    "msg": "Not in offboard, Returning control to the pilot"
                })
                return self.future

            elapsed_time = (self.clock.now() - start_time).nanoseconds / 1e9
            time_from_start = [0.0] + np.cumsum(dts).tolist()

            if elapsed_time >= time_from_start[-1]:
                # Fin de trayectoria
                self.future.set_result({
                    "success": True,
                    "msg": "Trajectory completed"
                })
                self.node.get_logger().debug("Trajectory finished")
                self.operation_timer.cancel()
                return

            index = max([i for i, t in enumerate(time_from_start) if t <= elapsed_time])

            target_local_pose = Pose()
            target_local_pose.position.x = poses[index].position.x - d_origin_x
            target_local_pose.position.y = poses[index].position.y - d_origin_y
            target_local_pose.position.z = poses[index].position.z - d_origin_z
     
            target_yaw = float(yaw[index])

            self.node.get_logger().debug(f"ned quaternion offboard_controller: {poses[index].orientation}")
            target_velocity = velocities[index]
            
            self.command_dispatcher.go_to(
                target_local_pose,
                # target_velocity,
                yaw=target_yaw
            )

        self.operation_timer = self.node.create_timer(self.command_period, callback, callback_group=self.operation_group)
        return self.future
    

    def follow_target(
        self,
        gps_origin: GeoPose,
        offset: np.ndarray,
        Kp: float = 1.5,
        Kd: float = 1.0
    ) -> Future:
        
        self.future = Future()

        reference = self.state_manager.state_repositories[States.LOCAL_POSITION].get()
        o_lat = reference.ref_lat
        o_lon = reference.ref_lon
        o_alt = reference.ref_alt

        d_origin_x, d_origin_y, d_origin_z = gps_to_ned(
            o_lat, 
            o_lon, 
            o_alt,
            gps_origin.position.latitude,
            gps_origin.position.longitude,
            gps_origin.position.altitude
        )

        def callback():
            if not self.check_offboard():
                self.operation_timer.cancel()
                self.future.set_result({
                    "success": True,
                    "msg": "Offboard lost, returning control"
                })
                return

            if not hasattr(self, "_target_pose") or self._target_pose is None:
                return

            current_pose = self.state_manager.state_repositories[States.LOCAL_POSITION].get()
            current_pos = np.array([current_pose.x, current_pose.y, current_pose.z])
            current_vel = np.array([current_pose.vx, current_pose.vy, current_pose.vz])

            # Offset rotado según el yaw del líder
            yaw = self._target_yaw  # yaw del líder en radianes
            R = np.array([
                [math.cos(yaw), -math.sin(yaw), 0.0],
                [math.sin(yaw),  math.cos(yaw), 0.0],
                [0.0, 0.0, 1.0]
            ])
            rotated_offset = R @ offset

            # Posición objetivo relativa al líder y rotada
            target_pos = np.array([
                self._target_pose.position.x - d_origin_x,
                self._target_pose.position.y - d_origin_y,
                self._target_pose.position.z - d_origin_z
            ]) + rotated_offset

            target_vel = np.array([
                self._target_velocity.linear.x,
                self._target_velocity.linear.y,
                self._target_velocity.linear.z
            ])
            target_vel = Kp * (target_pos - current_pos) + Kd * (target_vel - current_vel)

            target_pose = Pose()
            target_pose.position.x = target_pos[0]
            target_pose.position.y = target_pos[1]
            target_pose.position.z = target_pos[2]

            target_v = Twist()
            target_v.linear.x = target_vel[0]
            target_v.linear.y = target_vel[1]
            target_v.linear.z = target_vel[2]

            self.command_dispatcher.go_to(
                target_pose,
                velocity=target_v,
                yaw=self._target_yaw
            )

        self.operation_timer = self.node.create_timer(self.command_period, callback, callback_group=self.operation_group)
        return self.future


    def return_to_launch(self) -> None:

        self.future = Future()

        if not self.check_arm():
            self.node.get_logger().debug(f"Return to launch failed ")
            self.future.set_result({
                "success": False,
                "msg": "Return to launch failed"
            })
            return self.future
            
        def callback():
            if not self.check_arm():
                self.operation_timer.cancel()
                self.future.set_result({
                    "success": True,
                    "msg": "RTL completed"
                })
                return
        
        self.command_dispatcher.return_to_launch()
        self.operation_timer = self.node.create_timer(self.command_period, callback, callback_group=self.operation_group)
        return self.future
    

    def land(self) -> None:

        self.future = Future()

        if not self.check_arm():
            self.node.get_logger().debug(f"Land failed")
            self.future.set_result({
                "success": False,
                "msg": "Land failed"
            })
            return self.future
            
        def callback():
            if not self.check_arm():
                self.operation_timer.cancel()
                self.future.set_result({
                    "success": True,
                    "msg": "Vehicle landed"
                })
                return
        
        self.command_dispatcher.land()
        self.operation_timer = self.node.create_timer(self.command_period, callback, callback_group=self.operation_group)
        return self.future
    

    def cancel_operation(self) -> None:
        operation_state = OperationStatusState()
        operation_state.operation_state = OperationStatus.CANCELED
        self.state_manager.state_repositories[States.OPERATION_STATUS].update(operation_state)

        self.operation_timer.cancel()
        self.future.set_result({
                "success": True,
                "msg": "Operation canceled"
            })
        

    def offboard_mode(self):
        self.command_dispatcher.offboard()


    def offboard_heartbeat(self):
        self.command_dispatcher.offboard_heartbeat()


    def check_arm(self) -> bool:
        if self.state_manager.state_repositories[States.STATUS].get().arm_state is VehicleStatusStates.STATE_ARMED:
                return True
        return False
    
    
    def check_takeoff(self) -> bool:
        if self.state_manager.state_repositories[States.STATUS].get().nav_state is VehicleStatusStates.STATE_TAKEOFF:
                return True
        return False
    
    
    def check_hold(self) -> bool:
        if self.state_manager.state_repositories[States.STATUS].get().nav_state is VehicleStatusStates.STATE_HOLD:
            return True
        return False
    

    def check_failsafe(self) -> bool:
        return self.state_manager.state_repositories[States.STATUS].get().status_failsafe
            
      
    def check_offboard(self) -> bool:
        if self.state_manager.state_repositories[States.STATUS].get().nav_state is VehicleStatusStates.STATE_OFFBOARD:
            return True
        return False

    
    
    
