def verify_tree_sop_compliance( tree_dict):
    """
    Verifies if the behavior tree dictionary satisfies the SOP requirements.
    Returns (is_valid, missing_actions_list).
    """
    # SOP actions that must appear in the tree
    required_actions = {
        "takeOff1",
        "takeOff2",
        "acquireTarget",
        "cleanSurface",
        "attachSensor"
    }

    found_actions = set()

    def traverse(node):
        node_type = node.get("type", "").replace("-", "")
        # If this node is a required action, mark it as found
        if node_type in required_actions:
            found_actions.add(node_type)

         # Recursively check children if present
        for child in node.get("children", []):
            traverse(child)

    traverse(tree_dict)

    missing = required_actions - found_actions
    is_valid = len(missing) == 0

    if not is_valid:
        print(f"SOP Compliance Failed. Missing actions: {missing}")
    else:
        print("SOP Compliance Passed. All required actions present.")

    return is_valid, list(missing)


phase1_dict = {
        "type": "parallel",
        "children": [
            {
                "type": "selector",
                "children": [
                    {
                        "type": "sequence",
                        "children": [
                            {"type": "takeOff1"},
                            {"type": "acquireTarget"},
                        ]
                    },
                    {"type": "recovery1"}
                ]
            },
            {
                "type": "selector",
                "children": [
                    {
                        "type": "sequence",
                        "children": [
                            {"type": "takeOff2"},
                            {"type": "approachTarget"},
                            {"type": "attachSensor"}
                        ]
                    },
                    {"type": "recovery2"}
                ]
            }
        ]
    }


is_valid, missing = verify_tree_sop_compliance(phase1_dict)