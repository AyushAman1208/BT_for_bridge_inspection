from time import sleep
import time
import py_trees
import py_trees.behaviours
import py_trees.common
import logging
from drone_actions import SwarmConductor
import argparse
import sys
import re

from itertools import cycle, islice
import rclpy

py_trees.logging.level = py_trees.logging.Level.DEBUG
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

#######################################################
#use this pipeline to genenrate a behaviour tree using a gpt api.

# Use a pipeline as a high-level helper
# from transformers import pipeline

# pipe = pipeline("text-generation", model="openai-community/gpt2")
# sop = open("sop.txt", "r").read()
# format = open("bt_format.txt", "r").read()
# available_actions = open("drone_actions.txt", "r").read()
# prompt = '''
# Can you please create a behaviour tree based on the following standard operating procedure.

# Standard operating procedure:{sop}

# The available actions are:{available_actions}

# The format of the required behaviour tree is:{format}


# Please create a behaviour tree (tree diagram) based on the standard operating procedure, using the available actions and the format provided.

# '''



# output = pipe(prompt, max_length=5000, do_sample=True)
# gptOutput = open("gptOutput.txt", "w")
# gptOutput.write(output[0]['generated_text'])
# print(output)

###############################################################################################################################################################





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

    def createPhase1BehaviorTree(self,tree_in_dict_form):

        root = self.build_tree_from_dict(tree_in_dict_form[0])
        

        return root
    
    def build_tree_from_dict(self, tree_in_dict_form):
        # print(tree_in_dict_form)
        if tree_in_dict_form is None or len(tree_in_dict_form) == 0:
            return None
        
        curr_node = tree_in_dict_form['type']
        curr_node = curr_node.replace('-', '')
        print('Current node: ', curr_node)
        if curr_node == 'selector':
            children = tree_in_dict_form['children']
            selector = py_trees.composites.Selector(name=tree_in_dict_form['type'], memory=True)
            for child in children:
                child = self.build_tree_from_dict(child)
                if child is not None:
                    selector.add_child(child)
            return selector
        elif curr_node == 'sequence':
            print('reached here')
            print(tree_in_dict_form)
            children = tree_in_dict_form['children']
            sequence = py_trees.composites.Sequence(name=tree_in_dict_form['type'], memory=True)
            for child in children:
                child = self.build_tree_from_dict(child)
                if child is not None:
                    sequence.add_child(child)
            return sequence
        elif curr_node == 'takeOff1':
            return self.takeOff(1)
        elif curr_node == 'takeOff2':
            return self.takeOff(2)
        elif curr_node == 'acquireTarget':
            return self.acquireTarget(1)
        elif curr_node == 'approachTarget':
            return self.approachTarget(2)
        elif curr_node == 'cleanSurface':
            return self.cleanSurface(1)
        elif curr_node == 'attachSensor':
            return self.attachSensor(2)
        
        else:
            return None
        

            


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
    
    phase1 = BridgeInspectionPhase1(swarm)#
    ##############################

    def parse_indented_strings_to_tree(indented_strings):
        """
        Parses an array of indented strings into a hierarchical dictionary structure.
        
        Args:
            indented_strings: List of strings with leading spaces indicating indentation level
        
        Returns:
            A nested dictionary representing the hierarchy
        """
        root = {}
        stack = [(-1, root)]  # (indent_level, parent_dict)
        
        for line in indented_strings:
            # Count leading spaces to determine indentation level
            indent = len(line) - len(line.lstrip(' '))
            content = line.strip()
            
            # Find the appropriate parent by going up the stack
            while stack and stack[-1][0] >= indent:
                stack.pop()
            
            if not stack:
                raise ValueError("Invalid indentation structure")
            
            # Get the parent dictionary
            _, parent_dict = stack[-1]
            
            # Add current item to parent and push to stack
            if content not in parent_dict:
                parent_dict[content] = {}
            stack.append((indent, parent_dict[content]))
        
        return root

    def parse_behavior_tree(tree_str):
        lines = tree_str.strip().split('\n')
        return parse_indented_strings_to_tree(lines)
    




    def _parse_lines(lines):
        print(lines)
        if(len(lines) == 0):
            return []
        tree = []
        i = 0
        while i<len(lines):
            currType = lines[i].strip().replace('-', '')
            currIndent = len(lines[i].strip()) - len(lines[i].lstrip())
            children = []
            print(len(lines[i].strip()) - len(lines[i].lstrip()))
            while i<len(lines) and (len(lines[i].strip()) - len(lines[i].lstrip()) > currIndent):
                
                children.append(lines[i].strip())
                i = i+1
                print('present line: ', lines[i].strip())
            print(children)

            children = _parse_lines(children)
            tree.append({'type': currType, 'children': children})
            i = i+1

        print('here')
        print('*****************************************************************************')
        print(tree)



                
        
        return tree


    tree_str = open("gptOutput.txt", "r").read()
    tree_in_dictionary_form = tree_str.replace("-", "")
    # tree_in_dictionary_form = parse_behavior_tree(tree_str)
    tree_in_dictionary_form = [{
  "type": "sequence",
  "children": [
    {
      "type": "sequence",
      "children": [
        {"type": "takeOff1"},
        {"type": "go_off_board"},
        {"type": "go_to"},
        {"type": "await_spraying_command"}
      ]
    },
    {
      "type": "sequence",
      "children": [
        {"type": "takeOff2"},
        {"type": "go_off_board"},
        {"type": "find_attachment_point"},
        {"type": "send_sensor_attachment_location"}
      ]
    },
    {
      "type": "sequence",
      "children": [
        {"type": "go_to"},
        {"type": "clean_surface"},
        {"type": "spray_adhesive"}
      ]
    },
    {
      "type": "sequence",
      "children": [
        {"type": "go_to"},
        {"type": "expose_manipulator"},
        {"type": "align_manipulator"},
        {"type": "apply_constant_pressure"}
      ]
    }
  ]
}
]
    print(tree_in_dictionary_form)
        
    try:
        print("\n--- Starting Behavior Tree ---\n")
        tree = py_trees.trees.BehaviourTree(root=phase1.createPhase1BehaviorTree(tree_in_dictionary_form))
        tree.setup(timeout=15)
        
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
    # finally:
        # Proper cleanup
        # spinner_node.destroy_node()
        # rclpy.shutdown()

# === Main Execution ===
if __name__ == "__main__":
    main()
