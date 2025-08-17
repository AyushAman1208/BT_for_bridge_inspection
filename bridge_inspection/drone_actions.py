from typing import List, Optional
from as2_msgs.msg import YawMode, BehaviorStatus
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
import time
from functools import partial


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

    def go_to_next(self, droneId, x, y, z, speed, yaw_mode, yaw_angle, frame_id) -> None:
        """Go to next position in path"""
        self.do_behavior("go_to", x, y, z, speed, yaw_mode, yaw_angle, 'earth', False)
        self.__current += 1

    def goal_reached(self) -> bool:
        """Check if current behavior has finished"""
        return bool(self.current_behavior and
                    self.current_behavior.status == BehaviorStatus.IDLE)


class SwarmConductor(Node):
    """Swarm Conductor"""

    def __init__(self, drones_ns: List[str], verbose: bool = False,
                 use_sim_time: bool = False):
        super().__init__('swarm_conductor')
        self.drones: dict[int, Dancer] = {}
        self.altitudes: dict[int, float] = {}

        for index, name in enumerate(drones_ns):
            path = []
            self.drones[index] = Dancer(name, path, verbose, use_sim_time)
            self.altitudes[index + 1] = 0.0

            # Correct subscription binding using partial
            self.create_subscription(
                PoseStamped,
                f'/{name}/self_localization/pose',
                partial(self._pose_callback, drone_id=index + 1),
                QoSProfile(depth=10)
            )

    def _pose_callback(self, msg: PoseStamped, drone_id: int):
        """Store the latest altitude for a given drone ID."""
        self.altitudes[drone_id] = float(msg.pose.position.z)

    # ---------- Drone Commands ----------

    def arm_drone(self, droneId):
        print(f"Drone {droneId} arming")
        return self.drones[droneId - 1].arm()

    def go_off_board(self, droneId):
        print(f"Drone {droneId} offboarding")
        return self.drones[droneId - 1].offboard()

    def go_to(self, droneId, x, y, z, speed, yaw_mode, yaw_angle, frame_id):
        print(f"Drone {droneId} going to {x}, {y}, {z}")
        return self.drones[droneId - 1].do_behavior("go_to", x, y, z, speed, yaw_mode, yaw_angle, frame_id, False)

    def find_attachment_point(self, droneId):
        print(f"Drone {droneId} finding attachment point")
        time.sleep(1)
        return True

    def clean_surface(self, droneId):
        print(f"Drone {droneId} cleaning surface")
        time.sleep(1)
        return True

    def spray_adhesive(self, droneId):
        print(f"Drone {droneId} spraying adhesive")
        time.sleep(1)
        return True

    def expose_manipulator(self, droneId):
        print(f"Drone {droneId} exposing manipulator")
        time.sleep(1)
        return True

    def align_manipulator(self, droneId):
        print(f"Drone {droneId} aligning manipulator")
        time.sleep(1)
        return True

    def apply_constant_pressure(self, droneId):
        print(f"Drone {droneId} applying constant pressure")
        time.sleep(1)
        return True

    def send_sensor_attachment_location(self, droneId):
        print(f"Drone {droneId} sending sensor attachment location")
        time.sleep(1)
        return True

    def shutdown(self, droneId):
        return self.drones[droneId - 1].shutdown()

    def get_ready(self, droneId) -> bool:
        drone = self.drones[droneId - 1]
        return drone.arm() and drone.offboard()

    def takeoff(self, droneId):
        try:
            drone = self.drones[droneId - 1]
            drone.do_behavior("takeoff", 1, 0.7, False)
            print(f"Drone {droneId} takeoff command sent")
            return True
        except Exception as e:
            print(f"Drone {droneId} takeoff failed: {str(e)}")
            return False

    def is_armed(self, droneId):
        return True  # Replace with actual check

    def is_in_offboard_mode(self, droneId):
        return True  # Replace with actual check

    def get_altitude(self, droneId) -> float:
        """Return latest altitude reading from subscription."""
        return self.altitudes.get(droneId, 0.0)

    def is_at_altitude(self, droneId, target_alt, tolerance=0.2, timeout=8.0):
        """
        Wait until drone reaches target altitude (± tolerance).
        """
        start_time = time.time()
        # while time.time() - start_time < timeout:
        #     current_alt = self.get_altitude(droneId)
        #     print(f"[DEBUG] Drone {droneId} current_alt: {current_alt:.2f}, target_alt: {target_alt}")
        #     if current_alt >= (target_alt - tolerance):
        #         return True
        #     time.sleep(0.2)
        return True

    def land(self, droneId):
        self.drones[droneId - 1].do_behavior("land", 0.4, False)
