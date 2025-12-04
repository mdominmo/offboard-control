from rclpy.node import Node
from rclpy.task import Future
from offboard_control.domain.control.i_multi_offboard_controller import IMultiOffboardController
from offboard_control.domain.control.i_single_offboard_controller import ISingleOffboardController
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPose
from offboard_control.control import utils

from offboard_control.domain.constant.states import States

class MultiOffboardController(IMultiOffboardController):

    def __init__(
            self, 
            node: Node,
            controllers: list[ISingleOffboardController]
        ):
        
        self.node = node
        self.controllers = controllers
        self.command_period = .05


    def set_home_all(self, home: GeoPose):
        
        self.node.get_logger().debug(f"Setting home to all vehicles.")
        
        for controller in self.controllers:
            controller.set_home(home)


    def arm(self, ids: list[int]):
        pass
    

    def arm_all(self):
        
        self.node.get_logger().debug(f"Arming all vehicles")
        
        future = Future()
        futures = []

        for controller in self.controllers:
            futures.append(controller.arm())

        def callback():
            if all(future.done() for future in futures):
                timer.cancel()
                self.node.get_logger().debug(f"All vehicles armed")
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future
    
    def disarm_all(self):
        
        self.node.get_logger().debug(f"Disarming all vehicles")
        
        future = Future()
        futures = []

        for controller in self.controllers:
            futures.append(controller.disarm())

        def callback():
            if all(future.done() for future in futures):
                timer.cancel()
                self.node.get_logger().debug(f"All vehicles disarmed")
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future
    

    def disarm(self, ids: list[int]):
        pass

        
    def take_off(self, ids: list[int], height: float):
        pass


    def take_off_all(self, height=10.0):

        future = Future()
        futures = []

        self.node.get_logger().debug(f"Taking off all vehicles")

        for controller in self.controllers:
            futures.append(controller.take_off(height))

        def callback():

            if any(future.done() and future.result()["success"] == False for future in futures):
                self.node.get_logger().error(f"Multi vehicle operation failed.")
                future.set_result({
                    "success": False,
                    "msg": "Multi vehicle operation failed."
                })
                timer.cancel()
                return 

            if all(future.done() for future in futures):
                timer.cancel()
                self.node.get_logger().debug(f"All vehicles on air")
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future


    def hold(self, ids: list[int]):

        self.node.get_logger().debug(f"Transition to hold mode in {ids} vehicles")

        for id in ids:
            self.controllers[id].hold()

        self.node.get_logger().debug(f"{ids} vehicles in hold mode")


    def hold_all(self):

        self.node.get_logger().debug(f"Transition to hold mode in all vehicles")

        for controller in self.controllers:
            controller.hold()

        self.node.get_logger().debug(f"All vehicles in hold mode")
                

    def return_to_launch(self, ids: list[int]):
        pass


    def return_to_launch_all(self):

        future = Future()
        futures = []
        self.node.get_logger().debug(f"All vehicles returning to launch")

        for controller in self.controllers:
            futures.append(controller.return_to_launch())

        def callback():
            
            if any(future.done() and future.result()["success"] == False for future in futures):
                self.node.get_logger().error(f"Multi vehicle operation failed.")
                future.set_result({
                    "success": False,
                    "msg": "Multi vehicle operation failed."
                })
                timer.cancel()
                return 

            if all(future.done() for future in futures):
                self.node.get_logger().debug(f"All vehicles landed")
                self.hold_all()
                timer.cancel()
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                
        timer = self.node.create_timer(self.command_period, callback)
        return future
    
    
    def land(self, ids: list[int]):
        pass

    
    def land_all(self):

        future = Future()
        futures = []
        
        self.node.get_logger().debug(f"All vehicles landing")

        for controller in self.controllers:
            futures.append(controller.land())

        def callback():

            if any(future.done() and future.result()["success"] == False for future in futures):
                self.node.get_logger().error(f"Multi vehicle operation failed.")
                future.set_result({
                    "success": False,
                    "msg": "Multi vehicle operation failed."
                })
                timer.cancel()
                return 

            if all(future.done() for future in futures):
                timer.cancel()
                self.node.get_logger().debug(f"All vehicles landed")
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future
    

    def go_to(self, id: int, pose: GeoPose):
        
        future = None
        
        self.node.get_logger().debug(f"vehicle {id} going to {pose.position}")
        future = self.controllers[id].go_to(pose)

        def callback():

            if future.result["success"] == False:
                self.node.get_logger().error(f"Multi vehicle operation failed.")
                future.set_result({
                    "success": False,
                    "msg": "Multi vehicle operation failed."
                })
                timer.cancel()
                return 

            if future.done():
                timer.cancel()
                self.node.get_logger().debug(f"vehicle {id} reached waypoint")
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future
    
    
    def go_to_all(self, poses: list[GeoPose]):

        future = Future()
        futures = []
        
        self.node.get_logger().debug(f"going to {[pose.position for pose in poses]}")

        for controller, pose in zip(self.controllers, poses):
            futures.append(controller.go_to(pose))

        def callback():        

            if any(future.done() and future.result()["success"] == False for future in futures):
                self.node.get_logger().error(f"Multi vehicle operation failed.")
                future.set_result({
                    "success": False,
                    "msg": "Multi vehicle operation failed."
                })
                timer.cancel()
                return 

            if all(future.done() for future in futures):
                timer.cancel()
                self.node.get_logger().debug(f"All vehicles reached waypoints")
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future
    
    
    def go_to_local(
            self, 
            ids: list[int], 
            gps_origin: GeoPose, 
            poses: list[Pose],
            n_points = 80,
            speed = 2.0,
            final_yaw: list[float] = [],
            yaw_fraction = 0.25
        ):
        
        future = Future()
        futures = []
        
        self.node.get_logger().debug(f"vehicles {ids} going to {poses}")
        
        if not final_yaw:
            final_yaw = [float('nan')] * len(ids)

        for id, pose, yaw in zip(ids, poses, final_yaw):
            futures.append(self.controllers[id].go_to_local(gps_origin, pose, n_points, speed, yaw, yaw_fraction))

        def callback():

            if any(future.done() and future.result()["success"] == False for future in futures):
                self.node.get_logger().error(f"Multi vehicle operation failed.")
                future.set_result({
                    "success": False,
                    "msg": "Multi vehicle operation failed."
                })
                timer.cancel()
                return 
            
            if all(future.done() for future in futures):
                timer.cancel()
                self.node.get_logger().debug(f"vehicles {ids} reached positions")
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future
        
    
    def go_to_all_local(
            self,
            gps_origin: GeoPose, 
            poses: list[Pose],
            n_points = 80,
            speed = 2.0,
            final_yaw: list[float] = [],
            yaw_fraction = 0.25
        ):

        future = Future()
        futures = []
        
        self.node.get_logger().debug(f"going to {[pose.position for pose in poses]}")

        for controller, pose, yaw in zip(self.controllers, poses, final_yaw):
            futures.append(controller.go_to_local(gps_origin, pose, n_points, speed, yaw, yaw_fraction))

        def callback():
            
            if any(future.done() and future.result()["success"] == False for future in futures):
                self.node.get_logger().error(f"Multi vehicle operation failed.")
                future.set_result({
                    "success": False,
                    "msg": "Multi vehicle operation failed."
                })
                timer.cancel()
                return 
            
            if all(f.done() for f in futures):
                timer.cancel()
                self.node.get_logger().debug(f"All vehicles reached waypoints")
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future
    

    def trajectory_following(
            self, 
            ids: list[int], 
            trajectories
        ):
        
        futures = []
        future = Future()
        
        self.node.get_logger().debug(f"vehicles {ids} following trajectories {trajectories}")

        for id, trajectory in zip(ids, trajectories):

            futures.append(self.controllers[id].trajectory_following(trajectory[0], trajectory[1], trajectory[2], trajectory[3]))

        def callback():

            if any(future.done() and future.result()["success"] == False for future in futures):
                self.node.get_logger().error(f"Multi vehicle operation failed.")
                future.set_result({
                    "success": False,
                    "msg": "Multi vehicle operation failed."
                })
                timer.cancel()
                return 
            
            if all(future.done() for future in futures):
                timer.cancel()
                self.node.get_logger().info(f"vehicles {ids} trajectory finished")
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future
    

    def trajectory_following_all(
            self, 
            trajectories
        ):
        
        futures = []
        future = Future()
        
        self.node.get_logger().debug(f"All vehicles following trajectories")

        for controller, trajectory in zip(self.controllers, trajectories):
            futures.append(controller.trajectory_following(
                trajectory[0],
                trajectory[1],
                trajectory[2],
                trajectory[3]
            ))

        def callback(): 

            if any(future.done() and future.result()["success"] == False for future in futures):
                self.node.get_logger().error(f"Multi vehicle operation failed.")
                future.set_result({
                    "success": False,
                    "msg": "Multi vehicle operation failed."
                })
                timer.cancel()
                return 

            if all(future.done() for future in futures):
                timer.cancel()
                self.node.get_logger().debug(f"All vehicles finished trajectory")
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future
        
    
    def local_waypoint_following_all(
            self,
            gps_origin: GeoPose,
            waypoints: list[list[Pose]]
        ):

        future = Future()
        futures = []
        
        self.node.get_logger().debug(f"All vehicles following waypoints")

        for controller, waypoint_list in zip(self.controllers, waypoints):
            futures.append(controller.local_waypoint_following(gps_origin, waypoint_list))

        def callback():

            if any(future.done() and future.result()["success"] == False for future in futures):
                self.node.get_logger().error(f"Multi vehicle operation failed.")
                future.set_result({
                    "success": False,
                    "msg": "Multi vehicle operation failed."
                })
                timer.cancel()
                return 
            
            if all(future.done() for future in futures):
                timer.cancel()
                self.node.get_logger().debug(f"All vehicles finished waypoint following")
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future
    

    def leader_follower(
            self,
            formation_dist: float,
            leader_id = 0,
        ):

        future = Future()
        futures = []
        
        self.node.get_logger().debug(f"Vehicles performing leader follower")
        gps_origin = self.controllers[leader_id].state_manager.state_repositories[States.LOCAL_POSITION].get().get_gps_ref()
        
        offsets = utils.leader_follower_offsets(
            len(self.controllers), formation_dist
        )

        for i in range(len(self.controllers)):
            if i == leader_id:
                continue     
            futures.append(
                self.controllers[i].follow_target(
                    gps_origin,
                    offsets[i]
                ))

        def callback():
            
            l_pose = self.controllers[leader_id].state_manager.state_repositories[States.LOCAL_POSITION].get().get_pose()
            l_yaw = self.controllers[leader_id].state_manager.state_repositories[States.LOCAL_POSITION].get().heading
            l_twist = self.controllers[leader_id].state_manager.state_repositories[States.LOCAL_POSITION].get().get_twist()

            for controller in self.controllers:
                controller.set_external_target(l_pose, l_twist, l_yaw)

            if any(future.done() and future.result()["success"] == False for future in futures):
                self.node.get_logger().error(f"Multi vehicle operation failed.")
                future.set_result({
                    "success": False,
                    "msg": "Multi vehicle operation failed."
                })
                timer.cancel()
                return 
            
            if all(future.done() for future in futures):
                timer.cancel()
                self.node.get_logger().debug(f"All vehicles finished waypoint following")
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future
    
    
    def cancel_operation_all(self):
        
        self.node.get_logger().debug(f"Canceling all vehicles operations")

        for controller in self.controllers:
            controller.cancel_operation()


    def cancel_operation(self, ids: list[int]):
        
        self.node.get_logger().debug(f"Canceling {ids} vehicles operations")

        for id in ids:
            self.controllers[id].cancel_operation()


    def check_arm(self):

        future = Future()
        
        self.node.get_logger().debug(f"All vehicles checking arm")
        
        def callback(): 
            if all(controller.check_arm() for controller in self.controllers):
                timer.cancel()
                self.node.get_logger().debug(f"All vehicles armed")
                future.set_result({
                    "success": True,
                    "msg": "Multi vehicle operation perfomed."
                })
                return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future
    

    def check_offboard(self, ids=None):

        future = Future()
        
        self.node.get_logger().debug(f"All vehicles waiting offboard")
        
        def callback(): 

            if not ids:

                if all(controller.check_offboard() for controller in self.controllers):
                    timer.cancel()
                    self.node.get_logger().debug(f"All vehicles in offboard mode")
                    future.set_result({
                        "success": True,
                        "msg": "Multi vehicle operation perfomed."
                    })
                    return
            else:
                if all(self.controllers[id].check_offboard() for id in ids):
                    timer.cancel()
                    self.node.get_logger().debug(f"Selected vehicles in offboard mode")
                    future.set_result({
                        "success": True,
                        "msg": "Multi vehicle operation perfomed."
                    })
                    return
            
        timer = self.node.create_timer(self.command_period, callback)
        return future
    

    def offboard_mode_all(self):
        for controller in self.controllers:
            controller.offboard_mode()
    

    
    