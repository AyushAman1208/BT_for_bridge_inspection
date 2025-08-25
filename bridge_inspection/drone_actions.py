from typing import List, Optional
from math import radians, cos, sin
from as2_msgs.msg import YawMode
from as2_msgs.msg import BehaviorStatus
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from time import sleep
from rclpy.node import Node
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler #,GoalRejected, GoalFailed
from as2_msgs.msg import YawMode
class Dancer(DroneInterface):
    def __init__(self, namespace: str, path: list, verbose: bool = False,
                 use_sim_time: bool = False):
        super().__init__(namespace, verbose=verbose, use_sim_time=use_sim_time)

        self.__path = path
        self.__current = 0
        self.__speed = 0.5
        self.__yaw_mode = YawMode.PATH_FACING
        self.__yaw_angle = None
        self.__frame_id = "earth"

        self.current_behavior: Optional[BehaviorHandler] = None
        self.current_altitude = 0.0   # store latest altitude

        # Subscribe to pose updates
        self.create_subscription(
            PoseStamped,
            f"{namespace}/self_localization/pose",
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: PoseStamped):
        """Update altitude from pose"""
        self.current_altitude = msg.pose.position.z


    def reset(self) -> None:
        """Set current waypoint in path to start point"""
        self.__current = 0

    def do_behavior(self, beh, *args):
        print('***********************************************************************')
        print('inside do_behavior')
        print('behaviour: ',beh)
        print('args: ',args)
        method = getattr(self, beh)              # bound method (e.g., self.go_to)
        print('method: ',method)
        handler = method(*args)       
        print('handler: ',handler)           # BehaviorHandler
        self.current_behavior = method
        print('self.current_behavior: ',self.current_behavior)
        print('End of do_behavior')
        print('***********************************************************************')
        return handler


    def go_to_next(self,droneId, x, y, z, speed, yaw_mode, yaw_angle, frame_id) -> None:
        """Got to next position in path"""
        self.do_behavior("go_to", x, y, z, speed, yaw_mode, yaw_angle, 'earth', False)
        self.__current += 1

    def goal_reached(self) -> bool:
        """Check if current behavior has finished"""
        if not self.current_behavior:
            return False

        if self.current_behavior.status == BehaviorStatus.IDLE:
            return True
        return False
    


class SwarmConductor(Node):
    """Swarm Conductor"""

    def __init__(self, drones_ns: List[str], verbose: bool = False,
                 use_sim_time: bool = False):
        self.drones: dict[int, Dancer] = {}
        for index, name in enumerate(drones_ns):
            path = []
            self.drones[index] = Dancer(name, path, verbose, use_sim_time)

    ##************************************
    def arm_drone(self,droneId):
        print(f"Drone {droneId} arming")
        """Arm all drones in swarm"""
        drone = self.drones[droneId-1]
        arm_drone_status = drone.arm()
        print('arm_drone_status',arm_drone_status)
        print('current_behavior',drone.current_behavior)

        return arm_drone_status

    def go_off_board(self,droneId):
        """Offboard all drones in swarm"""
        print(f"Drone {droneId} offboarding")
        drone = self.drones[droneId-1]
        offboard_drone_status = drone.offboard()
        print('offboard_drone_status',offboard_drone_status)
        print('current_behavior',drone.current_behavior)
        return offboard_drone_status
    def go_to(self, droneId, x, y, z, speed, yaw_mode, yaw_angle, frame_id):
        drone = self.drones[droneId-1]

        # Normalize yaw parameters: prefer enums, angle None when not FIXED
        # yaw_mode = yaw_mode if isinstance(yaw_mode, int) else YawMode.PATH_FACING
        # yaw_angle = None if yaw_mode != YawMode.FIXED else (yaw_angle or 0.0)



        print('***********************************************************************')
        print(f"Drone {droneId} going to {x}, {y}, {z}")

        # 1st call: start once, then report RUNNING (by returning None)
        # print(drone.current_behavior.is_running())
        if drone.current_behavior is None or not drone.current_behavior.is_running():
            try:
                drone.go_to(x, y, z, speed, yaw_mode, yaw_angle, frame_id, False)
                handler = drone.do_behavior("go_to", x, y, z, speed, yaw_mode, yaw_angle, frame_id, False)
                return handler
                # if handler and drone.go_to.is_running(): return None  # tell ActionBehaviour: still running
                # print('pose',drone.go_to.__get_pose())
                # if handler:
                #     print('pose',drone.go_to.__get_pose()) 
                #     return True
                
            except BehaviorHandler.GoalRejected:
                print(f"[WARN] Drone {droneId} go_to goal rejected")
                drone.current_behavior = None
                return False

        # Subsequent ticks: if still running, keep RUNNING
        # print('##############################################################################')
        # print('drone.go_to.is_running()',drone.go_to.is_running())
        # print('##############################################################################')
        # return drone.go_to.result()

        # # Finished: convert handler result into True/False
        # try:

        #     result = drone.go_to.result()
        #     status = drone.go_to.result_status()
        #     if status != GoalStatus.STATUS_SUCCEEDED:
        #         self._node.get_logger().debug(
        #             f'Goal failed with status code: {status}')
        #         return False
        #     self._node.get_logger().debug(f'Result: {result}')
        #     return True
    
        # except BehaviorHandler.GoalFailed:
        #     return False
        # finally:
        #     drone.current_behavior.destroy()
        #     drone.current_behavior = None

    def find_attachment_point(self,droneId):
        print(f"Drone {droneId} finding attachment point")
        sleep(1)
        return True
    
    def clean_surface(self,droneId):
        print(f"Drone {droneId} cleaning surface")
        sleep(1)
        return True
    
    def spray_adhesive(self,droneId):
        print(f"Drone {droneId} spraying adhesive")
        sleep(1)
        return True
    
    def expose_manipulator(self,droneId):
        print(f"Drone {droneId} exposing manipulator")
        sleep(1)
        return True
    
    def align_manipulator(self,droneId):
        print(f"Drone {droneId} aligning manipulator")
        sleep(1)
        return True
    
    def apply_constant_pressure(self,droneId):
        print(f"Drone {droneId} applying constant pressure")
        sleep(1)
        return True
    
    def send_sensor_attachment_location(self,droneId):
        print(f"Drone {droneId} sending sensor attachment location")
        sleep(1)
        return True
    
    ##########################################################

    def shutdown(self,droneId):
        """Shutdown all drones in swarm"""
        drone = self.drones[droneId-1]
        return drone.shutdown()
    
    

    def get_ready(self,droneId) -> bool:
        """Arm and offboard for all drones in swarm"""
        success = True
        drone = self.drones[droneId-1]
        return drone.arm() and drone.offboard()

    def takeoff(self, droneId):
        drone = self.drones[droneId-1]
        print('current_behavior 1',drone.current_behavior)
        
        if drone.current_behavior is None or not drone.current_behavior.is_running():
            try:
                handler = drone.do_behavior("takeoff", 1.0, 0.7, False)  # height, speed, ignore_collisions
                print(handler)
                print('current_behavior 2',drone.current_behavior)
                print('current behaviour is running: ',drone.current_behavior.is_running())
                return handler
            except BehaviorHandler.GoalRejected:
                drone.current_behavior = None
                return False
        if drone.current_behavior.is_running():
            return None
        try:
            ok = drone.current_behavior.wait_to_result()
            return bool(ok)
        finally:
            drone.current_behavior.destroy()
            drone.current_behavior = None

    def land(self, droneId):
        drone = self.drones[droneId-1]
        # Stop any running behavior before sending land to avoid GoalRejected
        if drone.current_behavior and drone.current_behavior.is_running():
            drone.current_behavior.stop()
        if drone.current_behavior:
            drone.current_behavior.destroy()
            drone.current_behavior = None

        try:
            handler = drone.do_behavior("land", 0.4, False)
            return None
        except BehaviorHandler.GoalRejected:
            return False


    # Add these verification methods
    def is_armed(self, droneId):
        """Check if drone is armed (mock implementation)"""
        # In real implementation, check actual armed status
        return True

    def is_in_offboard_mode(self, droneId):
        """Check if drone is in offboard mode (mock implementation)"""
        # In real implementation, check actual mode
        return True

    def get_altitude(self, droneId):
        drone = self.drones[droneId-1]
        return drone.current_altitude
    
    def is_at_altitude(self, droneId, target_alt, tolerance=0.2):
        current_alt = self.get_altitude(droneId)
        print(f"[DEBUG] Drone {droneId} current_alt: {current_alt:.2f}, target_alt: {target_alt}")
        return abs(current_alt - target_alt) <= tolerance
