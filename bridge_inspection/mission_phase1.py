from time import sleep
import time
import py_trees
import py_trees.behaviours
import py_trees.common
import logging
from drone_actions import SwarmConductor, Dancer
import argparse
import sys

from itertools import cycle, islice
import rclpy

py_trees.logging.level = py_trees.logging.Level.DEBUG
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

class BatteryMonitor(py_trees.behaviour.Behaviour):
    def __init__(self, droneInterface, droneId, threshold=20.0):
        super().__init__(f"BatteryMonitor_{droneId}")
        self.droneInterface = droneInterface
        self.droneId = droneId
        self.threshold = threshold
        
    def update(self):
        battery_level = self.droneInterface.get_battery_level(self.droneId)
        if battery_level < self.threshold:
            logging.warning(f"Low battery on drone {self.droneId}: {battery_level}%")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
    
class ActionBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name, action_fn=None, max_retries=3, timeout=5.0):
        super().__init__(name)
        self.action_fn = action_fn
        self.max_retries = max_retries
        self.retry_count = 0
        self.timeout = timeout
        self.start_time = None
        self.action_started = False

    def initialise(self):
        self.start_time = time.time()
        self.retry_count = 0
        self.action_started = False
        logging.info(f"Starting: {self.name}")

    def update(self):
        if not self.action_started:
            self.action_started = True
            return py_trees.common.Status.RUNNING
            
        if time.time() - self.start_time > self.timeout:
            logging.error(f"Timeout in {self.name}")
            return py_trees.common.Status.FAILURE

        try:
            result = self.action_fn() if self.action_fn else True
            
            if result is True:
                return py_trees.common.Status.SUCCESS
            elif result is False:
                self.retry_count += 1
                if self.retry_count >= self.max_retries:
                    return py_trees.common.Status.FAILURE
                self.action_started = False  # Allow retry
                return py_trees.common.Status.RUNNING
            else:
                return py_trees.common.Status.RUNNING
                
        except Exception as e:
            logging.error(f"Error in {self.name}: {str(e)}")
            self.retry_count += 1
            if self.retry_count >= self.max_retries:
                return py_trees.common.Status.FAILURE
            self.action_started = False
            return py_trees.common.Status.RUNNING
        
class BridgeInspectionPhase1:
    def __init__(self,droneInterface):
        self.drones = []
        self.droneInterface = droneInterface
        self.tree = None

    # === Function that every behaviour will call ===
    def do_action(self,droneId):
        sleep(1)
        return True  

    # === Custom Action Node with Logging and Function Call ===
    
    def _verify_drone_methods(self, method_names):
        """Verify that the drone interface has the required methods"""
        for method_name in method_names:
            if not hasattr(self.droneInterface, method_name):
                raise AttributeError(
                    f"Drone interface is missing required method: {method_name}"
                )

    def takeOff(self, droneId):
        """Improved takeoff sequence with verification"""
        root = py_trees.composites.Sequence(name=f"TakeOff_{droneId}", memory=True)
        
        arm = ActionBehaviour(
            f"Arm_{droneId}", 
            lambda: self.droneInterface.arm_drone(droneId),
            max_retries=3,
            timeout=5.0
        )
        
        offboard = ActionBehaviour(
            f"Offboard_{droneId}", 
            lambda: self.droneInterface.go_off_board(droneId),
            max_retries=3,
            timeout=5.0
        )
        
        takeoff = ActionBehaviour(
            f"TakeOff_{droneId}", 
            lambda: self.droneInterface.takeoff(droneId),
            max_retries=3,
            timeout=10.0
        )
        
        # Add altitude verification
        altitude_check = ActionBehaviour(
            f"VerifyAltitude_{droneId}",
            lambda: self.droneInterface.is_at_altitude(droneId, 2.0),
            max_retries=10,
            timeout=20.0
        )
        
        root.add_children([arm, offboard, takeoff, altitude_check])
        return root

    def acquireTarget(self, droneId):
        root = py_trees.composites.Sequence(name=f"AcquireTarget_{droneId}", memory=True)
        
        # Verify these methods exist in SwarmConductor
        go_to = ActionBehaviour(
            "GoTo", 
            lambda: self.droneInterface.go_to(droneId, 1, 2, 3, 0.5, 0, 0, "earth"),
            max_retries=3,
            timeout=15.0
        )
        
        find_attachment = ActionBehaviour(
            "FindAttachmentPoint", 
            lambda: self.droneInterface.find_attachment_point(droneId),
            max_retries=5,
            timeout=20.0
        )
        
        root.add_children([go_to, find_attachment])
        return root
    
    def createRecoveryTree(self, droneId):
        recovery = py_trees.composites.Sequence(name=f"Recovery_{droneId}", memory=True)
        land = ActionBehaviour("Land", lambda: self.droneInterface.land(droneId))
        disarm = ActionBehaviour("Disarm", lambda: self.droneInterface.disarm_drone(droneId))
        recovery.add_children([land, disarm])
        return recovery

    def approachTarget(self,droneId):
        root = py_trees.composites.Sequence(name="Acquire Target", memory=True)
        goTo = ActionBehaviour("GoTo", lambda:self.droneInterface.go_to(droneId, 1, 2, 3, 0.5, 0, 0, "earth"))
        findAttachmentPoint = ActionBehaviour("Find Attachment Point", lambda:self.droneInterface.find_attachment_point(droneId))

        root.add_children([goTo, findAttachmentPoint])
        return root

    def cleanSurface(self,droneId):
        root = py_trees.composites.Sequence(name="Clean Surface", memory=True)
        cleanSurface = ActionBehaviour("Clean Surface", lambda:self.droneInterface.clean_surface(droneId))
        sprayAdhesive = ActionBehaviour("Spray Adhesive", lambda:self.droneInterface.spray_adhesive(droneId))
        sendSensorAttachmentLocation = ActionBehaviour("Send Sensor Attachment Location", lambda:self.droneInterface.send_sensor_attachment_location(droneId))

        root.add_children([cleanSurface, sprayAdhesive, sendSensorAttachmentLocation])
        return root

    def attachSensor(self,droneId):
        root = py_trees.composites.Sequence(name="Attach Sensor", memory=True)
        goTo = ActionBehaviour("GoTo", lambda:self.do_action(droneId))
        exposeManipulator = ActionBehaviour("Expose Manipulator", lambda:self.droneInterface.expose_manipulator(droneId))
        alignManipulator = ActionBehaviour("Align Manipulator", lambda:self.droneInterface.align_manipulator(droneId))
        applyConstantPressure = ActionBehaviour("Apply Constant Pressure", lambda:self.droneInterface.apply_constant_pressure(droneId))
        attachSensor = ActionBehaviour("Attach Sensor", lambda:self.do_action(droneId))

        root.add_children([goTo, exposeManipulator, alignManipulator, applyConstantPressure, attachSensor])
        return root

    def createPhase1BehaviorTree(self):
        droneId1 = 1
        droneId2 = 2

        root = py_trees.composites.Parallel(
            name="Phase1",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )

        try:
            drone1_tree = self._create_drone_sequence(droneId1, [
                ('takeOff', self.takeOff),
                ('acquireTarget', self.acquireTarget),
                ('cleanSurface', self.cleanSurface)
            ])
            
            drone2_tree = self._create_drone_sequence(droneId2, [
                ('takeOff', self.takeOff),
                ('approachTarget', self.approachTarget),
                ('attachSensor', self.attachSensor)
            ])
            
            root.add_children([drone1_tree, drone2_tree])
        except AttributeError as e:
            logging.error(f"Error creating behavior tree: {str(e)}")
            # Create a failure node if methods are missing
            root = py_trees.behaviours.Failure(name="TreeCreationFailed")

        return root

    def _create_drone_sequence(self, droneId, behaviors):
        """Helper to create a sequence with recovery for a drone"""
        selector = py_trees.composites.Selector(name=f"Drone{droneId}_WithRecovery", memory=True)
        sequence = py_trees.composites.Sequence(name=f"Drone{droneId}_Main", memory=True)
        
        for name, behavior_fn in behaviors:
            sequence.add_child(behavior_fn(droneId))
        
        selector.add_children([sequence, self.createRecoveryTree(droneId)])
        return selector
    def tick(self):
        self.tree.tick_tock(
            period_ms=1000,
            number_of_iterations=1,
            pre_tick_handler=None,
            post_tick_handler=None
        )


def confirm(msg: str = 'Continue') -> bool:
    """Confirm message"""
    confirmation = input(f"{msg}? (y/n): ")
    if confirmation == "y":
        return True
    return False

def main():
    parser = argparse.ArgumentParser(
        description='Single drone mission')

    parser.add_argument('-n', '--namespaces',
                        type=list,
                        default=['drone0', 'drone1'],
                        help='ID of the drone to be used in the mission')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')
    parser.add_argument('-s', '--use_sim_time',
                        action='store_true',
                        default=True,
                        help='Use simulation time')

    args = parser.parse_args()
    drones_namespace = args.namespaces
    verbosity = args.verbose
    use_sim_time = args.use_sim_time

    rclpy.init()
    swarm = SwarmConductor(
        drones_namespace,
        verbose=verbosity,
        use_sim_time=use_sim_time)
    
    # phase1 = BridgeInspectionPhase1(swarm)
    
    # try:
    #     print("\n--- Starting Behavior Tree ---\n")
    #     tree = py_trees.trees.BehaviourTree(root=phase1.createPhase1BehaviorTree())
    #     tree.setup(timeout=15)
        
    #     # Create a separate ROS node for spinning
    #     spinner_node = rclpy.create_node('behavior_tree_spinner')
    #     executor = rclpy.executors.SingleThreadedExecutor()
    #     executor.add_node(spinner_node)
        
    #     # Main execution loop
    #     while rclpy.ok():
    #         # Process ROS callbacks
    #         executor.spin_once(timeout_sec=0.1)
            
    #         # Tick the behavior tree
    #         tree.tick()
            
    #         # Check for completion
    #         if tree.root.status in [py_trees.common.Status.SUCCESS, 
    #                               py_trees.common.Status.FAILURE]:
    #             break
                
    #         # Small sleep to prevent busy waiting
    #         time.sleep(0.05)
            
    # except KeyboardInterrupt:
    #     print("\nMission interrupted by user")
    # finally:
    #     # Proper cleanup
    #     spinner_node.destroy_node()
    #     rclpy.shutdown()

# === Main Execution ===
if __name__ == "__main__":
    main()
