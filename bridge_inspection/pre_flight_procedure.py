import py_trees
import py_trees.behaviours
import py_trees.common
import logging

# === Logging Setup ===
py_trees.logging.level = py_trees.logging.Level.DEBUG
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')


# === Custom Action Node with Logging ===
class ActionBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name, success=True):
        super().__init__(name)
        self.success = success

    def initialise(self):
        logging.info(f"Initialising: {self.name}")

    def update(self):
        logging.info(f"Running: {self.name}")
        return py_trees.common.Status.SUCCESS if self.success else py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        logging.info(f"Terminating: {self.name} -> {new_status}")


# === Helper ===
def action(name, success=True):
    return ActionBehaviour(name=name, success=success)


# === Subtrees ===

def site_prepared_subtree():
    root = py_trees.composites.Selector(name="Site Prepared?", memory=False)

    ground_seq = py_trees.composites.Sequence(name="Ground System Setup", memory=False)
    ground_seq.add_children([
        action("Establish Ground System"),
        action("Calibrate Robot's Total Station")
    ])
    ground_ready = py_trees.composites.Selector(name="Ground System Established?", memory=False)
    ground_ready.add_child(ground_seq)

    risk_seq = py_trees.composites.Sequence(name="Risk Assessment Sequence", memory=False)
    risk_seq.add_children([
        action("Perform Risk Assessment"),
        action("Establish Safety Perimeter")
    ])
    risk_done = py_trees.composites.Selector(name="Risk Assessment Done?", memory=False)
    risk_done.add_child(risk_seq)

    root.add_children([
        ground_ready,
        action("Bridge Inspection Team Inserted"),
        action("Mark Inspection Zones"),
        risk_done
    ])
    return root


def system_check_subtree():
    root = py_trees.composites.Selector(name="System Check Complete?", memory=False)
    check_seq = py_trees.composites.Sequence(name="System Check Steps", memory=False)
    check_seq.add_children([
        action("Conduct Visual and Structural Inspection"),
        action("Verify Securing Mechanism on UAV 1"),
        action("Test Base Comms on UAV 1"),
        action("Test Comms Mod on UAV 1"),
        action("Test Base Comms on UAV 2"),
        action("Test Comms Mod on UAV 2"),
    ])
    root.add_child(check_seq)
    return root


def c2c_connected_subtree():
    root = py_trees.composites.Selector(name="C2C Connected?", memory=False)
    root.add_child(action("Connect to C2C"))
    return root


def uavs_connected_subtree():
    root = py_trees.composites.Selector(name="UAVs Connected?", memory=False)
    root.add_child(action("Connect to UAVs"))
    return root


def telemetry_verified_subtree():
    root = py_trees.composites.Selector(name="Telemetry Stream Verified?", memory=False)
    root.add_child(action("Verify Telemetry Stream"))
    return root


# === Full Tree ===

def create_preflight_behavior_tree():
    root = py_trees.composites.Sequence(name="Pre-Flight Procedures", memory=False)
    root.add_children([
        site_prepared_subtree(),
        system_check_subtree(),
        c2c_connected_subtree(),
        uavs_connected_subtree(),
        telemetry_verified_subtree()
    ])
    return root


# === Main ===

if __name__ == "__main__":
    tree = py_trees.trees.BehaviourTree(create_preflight_behavior_tree())
    py_trees.display.render_dot_tree(tree.root)

    print("\n--- Ticking the Tree ---\n")
    tree.tick_tock(
        period_ms=1000,
        number_of_iterations=1,
        pre_tick_handler=None,
        post_tick_handler=None
    )
