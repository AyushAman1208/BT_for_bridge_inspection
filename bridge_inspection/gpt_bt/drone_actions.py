
from typing import List, Optional
from math import radians, cos, sin
from as2_msgs.msg import YawMode
from as2_msgs.msg import BehaviorStatus
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from time import sleep
from rclpy.node import Node

class Dancer(DroneInterface):
    """Drone Interface extended with path to perform and async behavior wait"""

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

    def reset(self) -> None:
        """Set current waypoint in path to start point"""
        self.__current = 0

    def do_behavior(self, beh, *args) -> None:
        """Start behavior and save current to check if finished or not"""
        self.current_behavior = getattr(self, beh)
        self.current_behavior(*args)

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
        return drone.arm()

    def go_off_board(self,droneId):
        """Offboard all drones in swarm"""
        print(f"Drone {droneId} offboarding")
        drone = self.drones[droneId-1]
        return drone.offboard()
    def go_to(self,droneId, x, y, z, speed, yaw_mode, yaw_angle, frame_id):
        """Go to next position in path"""
        drone = self.drones[droneId-1]
        print(f"Drone {droneId} going to {x}, {y}, {z}")
        return drone.do_behavior("go_to", x, y, z, speed, yaw_mode, yaw_angle, frame_id, False)
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
        """Takeoff with proper return status"""
        try:
            drone = self.drones[droneId-1]
            drone.do_behavior("takeoff", 1, 0.7, False)
            print(f"Drone {droneId} takeoff command sent")
            return True
        except Exception as e:
            print(f"Drone {droneId} takeoff failed: {str(e)}")
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
        """Get current altitude (mock implementation)"""
        # In real implementation, get actual altitude
        # For now, return a positive value to simulate successful takeoff
        return 3.0  
    
    def is_at_altitude(self, droneId, target_alt, tolerance=1):
        current_alt = self.get_altitude(droneId)
        print(f"[DEBUG] Drone {droneId} current_alt: {current_alt}, target_alt: {target_alt}")
        return abs(current_alt - target_alt) <= tolerance

    def land(self,droneId):
        """Land swarm and wait for all drones"""
        drone = self.drones[droneId-1]
        drone.do_behavior("land", 0.4, False)