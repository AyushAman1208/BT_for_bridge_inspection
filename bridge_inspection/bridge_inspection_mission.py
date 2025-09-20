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
from as2_msgs.msg import YawMode
import py_trees.display as display
from py_trees import blackboard

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
    def __init__(self, name, action_fn=None, max_retries=25, timeout=50.0):
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
            # mark that we started and wait for the next tick so the action can be processed
            self.action_started = True
            return py_trees.common.Status.RUNNING
            
        # timeout check
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
                # allow another attempt on the next tick
                self.action_started = False
                return py_trees.common.Status.RUNNING
            else:
                # a non-boolean result means the action is still in progress
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
        self.waypoints = [
            {
                'x': 1,
                'y': -3.25,
                'z': 8
            },
            {
                'x': 1,
                'y': -3.25,
                'z': 8
            },
            {
                'x': 1,
                'y': -3.25,
                'z': 8
            }
        ]
        self.drone1_waypoint = self.waypoints[0]
        self.drone2_waypoint = self.waypoints[1]
        self.drone1_waypoint_index = 0
        


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
            
    def armDrone(self,droneId):

        root = py_trees.composites.Sequence(name=f"ArmDrone_{droneId}", memory=True)
        arm = ActionBehaviour(
            f"Arm_{droneId}", 
            lambda: self.droneInterface.arm_drone(droneId),
            max_retries=3,
            timeout=120.0
        )
        root.add_children([arm])
        return root
    
    def offboard(self,droneId):

        root = py_trees.composites.Sequence(name=f"Go_offboard_{droneId}", memory=True)
        offboard = ActionBehaviour(
            f"Offboard_{droneId}", 
            lambda: self.droneInterface.go_off_board(droneId),
            max_retries=3,
            timeout=120.0
        )
        root.add_children([offboard])
        return root
    def takeOff(self, droneId):
        """Improved takeoff sequence with verification"""
        # verify required methods for takeoff
        self._verify_drone_methods(['arm_drone', 'go_off_board', 'takeoff', 'is_at_altitude'])

        root = py_trees.composites.Sequence(name=f"TakeOff_{droneId}", memory=True)
        
        
        
        
        takeoff = ActionBehaviour(
            f"TakeOffCmd_{droneId}", 
            lambda: self.droneInterface.takeoff(droneId),
            max_retries=3,
            timeout=120.0
        )
        root.add_children([ takeoff])
        return root

    def acquireTarget(self, droneId):
        # verify required methods for acquireTarget
        self._verify_drone_methods(['go_to', 'find_attachment_point', 'is_at_altitude'])

        root = py_trees.composites.Sequence(name=f"AcquireTarget_{droneId}", memory=True)
        

        go_to = ActionBehaviour(
            "GoTo", 
            # in BridgeInspectionPhase1.acquireTarget / approachTarget
            lambda: self.droneInterface.go_to(droneId, self.drone1_waypoint['x'], self.drone1_waypoint['y'], self.drone1_waypoint['z'], 0.5, YawMode.PATH_FACING, None, "earth")
,
            max_retries=5,
            timeout=50.0
        )
        
        
        
        root.add_children([ go_to])
        return root
    def findAttachmentPoint(self,droneId):


        root = py_trees.composites.Sequence(name=f"FindAttachmentPoint_{droneId}", memory=True)
        find_attachment = ActionBehaviour(
            "FindAttachmentPoint", 
            lambda: self.droneInterface.find_attachment_point(droneId,self.drone1_waypoint['x'], self.drone1_waypoint['y'], self.drone1_waypoint['z']),
            max_retries=5,
            timeout=20.0
        )
        root.add_children([find_attachment])
        return root
    
    def createRecoveryTree(self, droneId):
        recovery = py_trees.composites.Sequence(name=f"Recovery_{droneId}", memory=True)
        land = ActionBehaviour("Land", lambda: self.droneInterface.land(droneId), timeout=150.0)
        disarm = ActionBehaviour("Disarm", lambda: self.droneInterface.disarm_drone(droneId), timeout=150.0)
        recovery.add_children([land, disarm])
        return recovery
    
    def updateTarget(self,droneId):
        if(self.drone1_waypoint_index < len(self.waypoints)-1):
            self.drone1_waypoint_index += 1
            self.drone1_waypoint = self.waypoints[self.drone1_waypoint_index]
            self.drone2_waypoint = self.waypoints[self.drone1_waypoint_index]
            return None
        else:
            return True

    def land(self, droneId):
        
        # verify required methods
        self._verify_drone_methods(['land', 'disarm_drone'])

        root = py_trees.composites.Sequence(name=f"Land_{droneId}", memory=True)
        updateTarget = ActionBehaviour("UpdateTarget", lambda: self.updateTarget(droneId))
        land = ActionBehaviour("Land", lambda: self.droneInterface.land(droneId))
        disarm = ActionBehaviour("Disarm", lambda: self.droneInterface.disarm_drone(droneId))
        root.add_children([updateTarget, land, disarm])
        return root

    def approachTarget(self,droneId):
        # verify required methods
        self._verify_drone_methods(['go_to', 'find_attachment_point', 'is_at_altitude'])

        root = py_trees.composites.Sequence(name=f"ApproachTarget_{droneId}", memory=True)
        # wait_air = WaitForAltitude(self.droneInterface, droneId, altitude=1.5, timeout=120.0)
        goTo = ActionBehaviour("GoTo", lambda:self.droneInterface.go_to(droneId,self.drone2_waypoint['x'], self.drone2_waypoint['y'], self.drone2_waypoint['z'], 0.5, 0, 0, "earth"), max_retries=5, timeout=120.0)
        
        root.add_children([ goTo])
        return root
    def findAttachmentPoint(self,droneId):
        root = py_trees.composites.Sequence(name=f"FindAttachmentPoint_{droneId}", memory=True)
        findAttachmentPoint = ActionBehaviour("FindAttachmentPoint", lambda:self.droneInterface.find_attachment_point(droneId, self.drone2_waypoint['x'], self.drone2_waypoint['y'], self.drone2_waypoint['z']), max_retries=5, timeout=210.0)

        root.add_children([findAttachmentPoint])
        return root

    def cleanSurface(self,droneId):
        root = py_trees.composites.Sequence(name=f"CleanSurface_{droneId}", memory=True)
        cleanSurface = ActionBehaviour("Clean Surface", lambda:self.droneInterface.clean_surface(droneId,self.drone1_waypoint['x'],self.drone1_waypoint['y'],self.drone1_waypoint['z']))
        
        root.add_children([cleanSurface])
        return root
    
    def sprayAdhesive(self,droneId):
        root = py_trees.composites.Sequence(name=f"SprayAdhesive_{droneId}", memory=True)
        sprayAdhesive = ActionBehaviour("Spray Adhesive", lambda:self.droneInterface.spray_adhesive(droneId,self.drone1_waypoint['x'],self.drone1_waypoint['y'],self.drone1_waypoint['z']))

        root.add_children([sprayAdhesive])
        return root
    def sendSensorAttachmentLocation(self,droneId):
        root = py_trees.composites.Sequence(name=f"SendSensorAttachmentLocation_{droneId}", memory=True)
        sendSensorAttachmentLocation = ActionBehaviour("Send Sensor Attachment Location", lambda:self.droneInterface.send_sensor_attachment_location(droneId,self.drone1_waypoint['x'],self.drone1_waypoint['y'],self.drone1_waypoint['z']))

        root.add_children([sendSensorAttachmentLocation])
        return root
    def exposeManipulator(self,droneId):
        root = py_trees.composites.Sequence(name=f"ExposeManipulator_{droneId}", memory=True)
        exposeManipulator = ActionBehaviour("Expose Manipulator", lambda:self.droneInterface.expose_manipulator(droneId,self.drone2_waypoint['x'], self.drone2_waypoint['y'], self.drone2_waypoint['z']))
        
        root.add_children([exposeManipulator])
        return root
    def alignManipulator(self,droneId):
        root = py_trees.composites.Sequence(name=f"AlignManipulator_{droneId}", memory=True)
        alignManipulator = ActionBehaviour("Align Manipulator", lambda:self.droneInterface.align_manipulator(droneId,self.drone2_waypoint['x'], self.drone2_waypoint['y'], self.drone2_waypoint['z'] ))
        
        root.add_children([alignManipulator])
        return root
    
    def applyConstantPressure(self,droneId):
        root = py_trees.composites.Sequence(name=f"ApplyConstantPressure_{droneId}", memory=True)
        applyConstantPressure = ActionBehaviour("Apply Constant Pressure", lambda:self.droneInterface.apply_constant_pressure(droneId,self.drone2_waypoint['x'], self.drone2_waypoint['y'], self.drone2_waypoint['z']))
        
        root.add_children([applyConstantPressure])
        return root
    def attachSensor(self,droneId):
        root = py_trees.composites.Sequence(name=f"AttachSensor_{droneId}", memory=True)
        attachSensor = ActionBehaviour("Attach Sensor", lambda:self.do_action(droneId))

        root.add_children([attachSensor])
        return root

    def createPhase1BehaviorTree(self):
        droneId1 = 1
        droneId2 = 2

        root = py_trees.composites.Parallel(
            name="Phase1",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
            # memory=False
        )

        try:
            # Verify top-level expected methods once before building the trees
            self._verify_drone_methods(['arm_drone', 'go_off_board', 'takeoff', 'is_at_altitude', 'go_to', 'find_attachment_point'])

            drone1_tree = self._create_drone_sequence(droneId1, [
                ('armDrone', self.armDrone),
                ('goOffboard', self.offboard),
                ('takeOff', self.takeOff),
                ('acquireTarget', self.acquireTarget),
                ('findAttachmentPoint', self.findAttachmentPoint),
                ('cleanSurface', self.cleanSurface),
                ('sprayAdhesive', self.sprayAdhesive),
                ('sendSensorAttachmentLocation', self.sendSensorAttachmentLocation),
                ('land', self.land)
            ])
            
            drone2_tree = self._create_drone_sequence(droneId2, [
                ('armDrone', self.armDrone),
                ('goOffboard', self.offboard),
                ('takeOff', self.takeOff),
                ('approachTarget', self.approachTarget),
                ('findAttachmentPoint', self.findAttachmentPoint),
                ('exposeManipulator', self.exposeManipulator),
                ('alignManipulator', self.alignManipulator),
                ('applyConstantPressure', self.applyConstantPressure),
                ('attachSensor', self.attachSensor),
                ('land', self.land)
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
    
    phase1 = BridgeInspectionPhase1(swarm)
    
    try:
        print("\n--- Starting Behavior Tree ---\n")
        tree = py_trees.trees.BehaviourTree(root=phase1.createPhase1BehaviorTree())
        tree.setup(timeout=150)
        dot_graph = py_trees.display.render_dot_tree(tree.root)
        print(dot_graph)
        
        # Create a separate ROS node for spinning
        spinner_node = rclpy.create_node('behavior_tree_spinner')
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(spinner_node)
        
        # Main execution loop
        while rclpy.ok():
            # Process ROS callbacks
            executor.spin_once(timeout_sec=0.1)
            
            # Tick the behavior tree
            tree.tick()
            
            # Check for completion
            if tree.root.status in [py_trees.common.Status.SUCCESS, 
                                  py_trees.common.Status.FAILURE]:
                break
                
            # Small sleep to prevent busy waiting
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nMission interrupted by user")
    finally:
        # Proper cleanup
        spinner_node.destroy_node()
        rclpy.shutdown()

# === Main Execution ===
if __name__ == "__main__":
    main()
