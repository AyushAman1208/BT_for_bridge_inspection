import yaml
import json
import math
import argparse
import os
import random
from jinja2 import Template

# SDF Templates
SDF_TREE_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{{ model_name }}">
    <static>true</static>
    <link name="tree">
      <visual name="tree_visual">
        <geometry>
          <cylinder>
            <radius>{{ diameter / 2 }}</radius>
            <length>{{ height }}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.3 0.2 0.1 1.0</ambient>
          <diffuse>0.3 0.2 0.1 1.0</diffuse>
        </material>
      </visual>
      <collision name="tree_collision">
        <geometry>
          <cylinder>
            <radius>{{ diameter / 2 }}</radius>
            <length>{{ height }}</length>
          </cylinder>
        </geometry>
      </collision>
      <pose>0 0 {{ height / 2 }} 0 0 0</pose>
    </link>
  </model>
</sdf>
"""

SDF_FLOOR_TILE_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{{ model_name }}">
    <static>true</static>
    <link name="floor_tile">
      <visual name="tile_visual">
        <geometry>
          <box>
            <size>{{ width }} {{ length }} 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>{{ colour }}</ambient>
          <diffuse>{{ colour }}</diffuse>
        </material>
      </visual>
      <pose>0 0 0.005 0 0 0</pose>
    </link>
  </model>
</sdf>
"""

SDF_DYNAMIC_OBJECT_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{{ model_name }}">
    <static>false</static>
    <link name="dynamic_object">
      <inertial>
        <mass>100</mass>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>{{ width }} {{ length }} {{ height }}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{{ width }} {{ length }} {{ height }}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.3 0.5 1.0</ambient>
          <diffuse>0.7 0.3 0.5 1.0</diffuse>
        </material>
      </visual>
    </link>
    <plugin filename="libDynamicMovingObjects" name="gzplugin::DynamicMovingObjects">
      <boundary_min_x>{{ boundary_min_x }}</boundary_min_x>
      <boundary_max_x>{{ boundary_max_x }}</boundary_max_x>
      <boundary_min_y>{{ boundary_min_y }}</boundary_min_y>
      <boundary_max_y>{{ boundary_max_y }}</boundary_max_y>
      <movement_velocity>{{ movement_velocity }}</movement_velocity>
      <initial_angle>{{ initial_angle }}</initial_angle>
    </plugin>
  </model>
</sdf>
"""

SDF_WINDOW_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{{ model_name }}">
    <static>true</static>
    <link name="bottom_wall">
      <visual name="bottom_visual">
        <geometry>
          <box>
            <size>{{ room_width }} {{ wall_depth }} {{ window_bottom }}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1.0</ambient>
          <diffuse>0.5 0.5 0.5 1.0</diffuse>
        </material>
      </visual>
      <collision name="bottom_collision">
        <geometry>
          <box>
            <size>{{ room_width }} {{ wall_depth }} {{ window_bottom }}</size>
          </box>
        </geometry>
      </collision>
      <pose>0 0 {{ window_bottom / 2 }} 0 0 0</pose>
    </link>
    <link name="top_wall">
      <visual name="top_visual">
        <geometry>
          <box>
            <size>{{ room_width }} {{ wall_depth }} {{ wall_top }}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1.0</ambient>
          <diffuse>0.5 0.5 0.5 1.0</diffuse>
        </material>
      </visual>
      <collision name="top_collision">
        <geometry>
          <box>
            <size>{{ room_width }} {{ wall_depth }} {{ wall_top }}</size>
          </box>
        </geometry>
      </collision>
      <pose>0 0 {{ window_bottom + window_height + (wall_top / 2) }} 0 0 0</pose>
    </link>
    <link name="left_wall">
      <visual name="left_visual">
        <geometry>
          <box>
            <size>{{ left_width }} {{ wall_depth }} {{ room_height }}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1.0</ambient>
          <diffuse>0.5 0.5 0.5 1.0</diffuse>
        </material>
      </visual>
      <collision name="left_collision">
        <geometry>
          <box>
            <size>{{ left_width }} {{ wall_depth }} {{ room_height }}</size>
          </box>
        </geometry>
      </collision>
      <pose>{{ left_x }} 0 {{ room_height / 2 }} 0 0 0</pose>
    </link>
    <link name="right_wall">
      <visual name="right_visual">
        <geometry>
          <box>
            <size>{{ right_width }} {{ wall_depth }} {{ room_height }}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1.0</ambient>
          <diffuse>0.5 0.5 0.5 1.0</diffuse>
        </material>
      </visual>
      <collision name="right_collision">
        <geometry>
          <box>
            <size>{{ right_width }} {{ wall_depth }} {{ room_height }}</size>
          </box>
        </geometry>
      </collision>
      <pose>{{ right_x }} 0 {{ room_height / 2 }} 0 0 0</pose>
    </link>
  </model>
</sdf>
"""

SDF_BRIDGE_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{{ model_name }}">
    <static>true</static>
    
    <!-- Main Bridge Deck -->
    <link name="deck">
      <visual name="deck_visual">
        <geometry>
          <box>
            <size>{{ length }} {{ width }} {{ thickness }}</size>
          </box>
        </geometry>
        <material>
          <ambient>{{ deck_color }}</ambient>
          <diffuse>{{ deck_color }}</diffuse>
        </material>
      </visual>
      <collision name="deck_collision">
        <geometry>
          <box>
            <size>{{ length }} {{ width }} {{ thickness }}</size>
          </box>
        </geometry>
      </collision>
      <pose>0 0 {{ height + thickness/2 }} 0 0 0</pose>
    </link>

    <!-- Left Support Pillar -->
    <link name="left_pillar">
      <visual name="left_pillar_visual">
        <geometry>
          <box>
            <size>{{ pillar_width }} {{ pillar_width }} {{ height }}</size>
          </box>
        </geometry>
        <material>
          <ambient>{{ pillar_color }}</ambient>
          <diffuse>{{ pillar_color }}</diffuse>
        </material>
      </visual>
      <collision name="left_pillar_collision">
        <geometry>
          <box>
            <size>{{ pillar_width }} {{ pillar_width }} {{ height }}</size>
          </box>
        </geometry>
      </collision>
      <pose>{{ -length/2 + pillar_width/2 }} 0 {{ height/2 }} 0 0 0</pose>
    </link>

    <!-- Right Support Pillar -->
    <link name="right_pillar">
      <visual name="right_pillar_visual">
        <geometry>
          <box>
            <size>{{ pillar_width }} {{ pillar_width }} {{ height }}</size>
          </box>
        </geometry>
        <material>
          <ambient>{{ pillar_color }}</ambient>
          <diffuse>{{ pillar_color }}</diffuse>
        </material>
      </visual>
      <collision name="right_pillar_collision">
        <geometry>
          <box>
            <size>{{ pillar_width }} {{ pillar_width }} {{ height }}</size>
          </box>
        </geometry>
      </collision>
      <pose>{{ length/2 - pillar_width/2 }} 0 {{ height/2 }} 0 0 0</pose>
    </link>

    {% if railings %}
    <!-- Left Railing -->
    <link name="left_railing">
      <visual name="left_railing_visual">
        <geometry>
          <box>
            <size>{{ length }} {{ railing_thickness }} {{ railing_height }}</size>
          </box>
        </geometry>
        <material>
          <ambient>{{ railing_color }}</ambient>
          <diffuse>{{ railing_color }}</diffuse>
        </material>
      </visual>
      <collision name="left_railing_collision">
        <geometry>
          <box>
            <size>{{ length }} {{ railing_thickness }} {{ railing_height }}</size>
          </box>
        </geometry>
      </collision>
      <pose>0 {{ width/2 - railing_thickness/2 }} {{ height + thickness + railing_height/2 }} 0 0 0</pose>
    </link>

    <!-- Right Railing -->
    <link name="right_railing">
      <visual name="right_railing_visual">
        <geometry>
          <box>
            <size>{{ length }} {{ railing_thickness }} {{ railing_height }}</size>
          </box>
        </geometry>
        <material>
          <ambient>{{ railing_color }}</ambient>
          <diffuse>{{ railing_color }}</diffuse>
        </material>
      </visual>
      <collision name="right_railing_collision">
        <geometry>
          <box>
            <size>{{ length }} {{ railing_thickness }} {{ railing_height }}</size>
          </box>
        </geometry>
      </collision>
      <pose>0 {{ -width/2 + railing_thickness/2 }} {{ height + thickness + railing_height/2 }} 0 0 0</pose>
    </link>
    {% endif %}
  </model>
</sdf>
"""

MODEL_CONFIG_TEMPLATE = """<?xml version="1.0" ?>
<model>
  <name>{{ model_name }}</name>
  <version>1.0</version>
  <sdf version="1.6">{{ model_name }}.sdf</sdf>
  <author>
    <name>Aerostack2 Team</name>
    <email>aerostack@example.com</email>
  </author>
  <description>
    {{ model_name }}
  </description>
</model>
"""

BRIDGE_MODEL_CONFIG_TEMPLATE = """<?xml version="1.0" ?>
<model>
  <name>{{ model_name }}</name>
  <version>1.0</version>
  <sdf version="1.6">{{ model_name }}.sdf</sdf>
  <author>
    <name>Aerostack2 Team</name>
    <email>aerostack@example.com</email>
  </author>
  <description>
    A bridge structure with deck and support pillars. 
    Length: {{ length }}m, Width: {{ width }}m, Height: {{ height }}m
  </description>
</model>
"""

def generate_window_model(output_dir, model_name, 
                          window_x, window_y, 
                          window_width, window_height, window_bottom,
                          room_width, room_height, 
                          wall_width, wall_depth):
    """Generate SDF and model.config for a cuboid."""
    model_dir = os.path.join(output_dir, model_name)
    os.makedirs(model_dir, exist_ok=True)

    wall_top = room_height - (window_bottom + window_height)
    left_width = (window_x - (window_width / 2)) + (room_width / 2)
    right_width = (room_width / 2) - (window_x + (window_width / 2))
    left_x = window_x - (window_width / 2) - (left_width / 2)
    right_x = window_x + (window_width / 2) + (right_width / 2)
    
    # Render SDF file
    sdf_content = Template(SDF_WINDOW_TEMPLATE).render(
        model_name=model_name,
        window_x=window_x, window_y=window_y, 
        window_width=window_width, window_height=window_height,
        window_bottom=window_bottom, 
        left_width=left_width, left_x=left_x, 
        right_width=right_width, right_x=right_x,
        room_height=room_height, room_width=room_width,
        wall_width=wall_width, wall_depth=wall_depth, 
        wall_top=wall_top
    )
    with open(os.path.join(model_dir, f"{model_name}.sdf"), "w") as sdf_file:
        sdf_file.write(sdf_content)
    
    # Render model.config
    config_content = Template(MODEL_CONFIG_TEMPLATE).render(
        model_name=model_name,
    )
    with open(os.path.join(model_dir, "model.config"), "w") as config_file:
        config_file.write(config_content)

def generate_tree_model(output_dir, model_name, diameter, height):
    """Generate SDF and model.config for a cuboid."""
    model_dir = os.path.join(output_dir, model_name)
    os.makedirs(model_dir, exist_ok=True)
    
    # Render SDF file
    sdf_content = Template(SDF_TREE_TEMPLATE).render(
        model_name=model_name, diameter=diameter, height=height
    )
    with open(os.path.join(model_dir, f"{model_name}.sdf"), "w") as sdf_file:
        sdf_file.write(sdf_content)
    
    # Render model.config
    config_content = Template(MODEL_CONFIG_TEMPLATE).render(
        model_name=model_name,
    )
    with open(os.path.join(model_dir, "model.config"), "w") as config_file:
        config_file.write(config_content)

def generate_dynamic_object_model(output_dir, model_name, size, boundaries, velocity, angle):
    """
    Generate an SDF file and model.config for a dynamic object.

    :param output_dir: Directory where model files will be saved.
    :param model_name: Name of the model.
    :param size: Tuple (width, length, height) defining the object size.
    :param boundaries: Dict with keys min_x, max_x, min_y, max_y.
    :param velocity: Movement velocity of the object.
    :param angle: Initial angle of the object in radians.
    """
    model_dir = os.path.join(output_dir, model_name)
    os.makedirs(model_dir, exist_ok=True)
    
    # Render SDF file
    sdf_content = Template(SDF_DYNAMIC_OBJECT_TEMPLATE).render(
        model_name=model_name,
        width=size[0], length=size[1], height=size[2],
        boundary_min_x=boundaries["min_x"],
        boundary_max_x=boundaries["max_x"],
        boundary_min_y=boundaries["min_y"],
        boundary_max_y=boundaries["max_y"],
        movement_velocity=velocity,
        initial_angle=angle
    )
    
    with open(os.path.join(model_dir, f"{model_name}.sdf"), "w") as sdf_file:
        sdf_file.write(sdf_content)

    # Render model.config
    config_content = Template(MODEL_CONFIG_TEMPLATE).render(
        model_name=model_name,
    )
    with open(os.path.join(model_dir, "model.config"), "w") as config_file:
        config_file.write(config_content)

def generate_floor_model(output_dir, model_name, size, colour):
    """Generate SDF and model.config for a cuboid."""
    model_dir = os.path.join(output_dir, model_name)
    os.makedirs(model_dir, exist_ok=True)
    
    # Render SDF file
    colours = " ".join(str(x) for x in colour)
    sdf_content = Template(SDF_FLOOR_TILE_TEMPLATE).render(
        model_name=model_name, width=size[0], length=size[1], colour=colours
    )
    with open(os.path.join(model_dir, f"{model_name}.sdf"), "w") as sdf_file:
        sdf_file.write(sdf_content)
    
    # Render model.config
    config_content = Template(MODEL_CONFIG_TEMPLATE).render(
        model_name=model_name,
    )
    with open(os.path.join(model_dir, "model.config"), "w") as config_file:
        config_file.write(config_content)

def generate_bridge_model(output_dir, model_name, length, width, height, thickness, 
                        pillar_width,
                        #   deck_color, pillar_color, 
                        #   railings=False,
                        # railing_thickness=0.1, railing_height=0.3, railing_color="0.8 0.8 0.8 1.0"
                        ):
    """
    Generate SDF and model.config for a bridge structure.
    """
    model_dir = os.path.join(output_dir, model_name)
    os.makedirs(model_dir, exist_ok=True)
    
    # Render SDF file
    sdf_content = Template(SDF_BRIDGE_TEMPLATE).render(
        model_name=model_name,
        length=length,
        width=width,
        height=height,
        thickness=thickness,
        pillar_width=pillar_width,
        # deck_color=deck_color,
        # pillar_color=pillar_color,
        # railings=railings,
        # railing_thickness=railing_thickness,
        # railing_height=railing_height,
        # railing_color=railing_color
    )
    with open(os.path.join(model_dir, f"{model_name}.sdf"), "w") as sdf_file:
        sdf_file.write(sdf_content)
    
    # Render model.config
    config_content = Template(BRIDGE_MODEL_CONFIG_TEMPLATE).render(
        model_name=model_name,
        length=length,
        width=width,
        height=height
    )
    with open(os.path.join(model_dir, "model.config"), "w") as config_file:
        config_file.write(config_content)

# Read the YAML scenario file
def read_yaml(file_path):
    with open(file_path, 'r') as file:
        scenario = yaml.safe_load(file)
    return scenario

def generate_as2_windows_config(scenario, output_model_folder):
    objects = []
    
    window_list = scenario["stage2"]["windows"]
    stage_center = scenario["stage2"]["stage_center"]

    # Add obstacles as objects
    for key, w in window_list.items():
        model_name = f"window_{key}"
        objects.append({
            "model_type": model_name,
            "model_name": model_name,
            "xyz": [stage_center[0], w["center"][1] + stage_center[1], 0.0],
        })
        generate_window_model(
            output_model_folder, model_name, 
            window_x=w["center"][0], window_y=w["center"][1], 
            window_width=w["gap_width"], window_height=w["height"], window_bottom=w["distance_floor"],
            room_height=scenario["stage2"]["room_height"], room_width=scenario["stage_size"][1],
            wall_width=scenario["stage_size"][1],
            wall_depth=w["thickness"])
    return objects  

def generate_as2_forest_config(scenario, output_model_folder):
    objects = []

    stage = scenario.get("stage3", {})
    stage_center = stage["stage_center"]

    model_name = "tree_column"
    generate_tree_model(output_model_folder, model_name, stage["obstacle_diameter"], stage["obstacle_height"])

    for key, coord in enumerate(stage["obstacles"]):
        objects.append({
            "model_type": model_name,
            "model_name": f"tree_{key}",
            "xyz": [coord[0] + stage_center[0], coord[1] + stage_center[1], 0.0],
        })
    return objects

def generate_as2_dynamic_objects_config(scenario, output_model_folder):
    objects = []
    size = scenario["stage_size"]
    stage = scenario["stage4"]

    for i in range(stage["num_obstacles"]):
        center = stage["stage_center"]

        # Get random start location relative to stage center and stage size
        loc = [l + (random.random() * s) - (s/2.0) for s, l in zip(size, center)]
        
        boundaries = {
            "min_x": center[0] - size[0]/2.0,
            "min_y": center[1] - size[1]/2.0,
            "max_x": center[0] + size[0]/2.0,
            "max_y": center[1] + size[1]/2.0
        }

        object_size = [stage["obstacle_diameter"], stage["obstacle_diameter"], stage["obstacle_height"]]

        model_name = f"dynamic_obstacle_{i}"
        generate_dynamic_object_model(output_model_folder, model_name, 
                                      object_size, boundaries=boundaries, 
                                      velocity=stage["obstacle_velocity"],
                                      angle=random.random() * 2 * math.pi)
        objects.append({
            "model_type": model_name,
            "model_name": model_name,
            "xyz": [loc[0], loc[1], 0.0],
        })
    return objects

def generate_as2_floor_tiles_config(scenario, output_model_folder):
    objects = []
    size = scenario["stage_size"]

    mapping = {
        "stage1": (1.0, 0.0, 0.4, 0.5),
        "stage2": (0.1, 1.0, 0.4, 0.5),
        "stage3": (0.1, 0.0, 0.5, 0.5),
        "stage4": (1.0, 0.0, 1.0, 0.5),
    }
    for stage, colour in mapping.items():
      if stage in scenario:
        loc = scenario[stage]["stage_center"]
        model_name =  f"floor_{stage}"
        generate_floor_model(output_model_folder, model_name, size, colour)
        objects.append({
            "model_type": model_name,
            "model_name": model_name,
            "xyz": [loc[0], loc[1], 0.0],
        })
    return objects

def generate_as2_bridge_config(scenario, output_model_folder):
    objects = []
    
    if "stage_bridge" not in scenario:
        return objects
        
    bridge_cfg = scenario["stage_bridge"]["bridge"]
    stage_center = scenario["stage_bridge"]["stage_center"]
    
    model_name = "scenario_bridge"
    generate_bridge_model(
        output_model_folder,
        model_name,
        length=bridge_cfg["length"],
        width=bridge_cfg["width"],
        height=bridge_cfg["height"],
        thickness=bridge_cfg["thickness"],
        pillar_width=bridge_cfg["pillar_width"],
        # deck_color=" ".join(map(str, bridge_cfg["deck_color"])),
        # pillar_color=" ".join(map(str, bridge_cfg["pillar_color"])),
        # railings=bridge_cfg.get("railings", False),
        # railing_thickness=bridge_cfg.get("railing_thickness", 0.1),
        # railing_height=bridge_cfg.get("railing_height", 0.3),
        # railing_color=" ".join(map(str, bridge_cfg.get("railing_color", [0.8, 0.8, 0.8, 1.0])))
        )
    
    objects.append({
        "model_type": model_name,
        "model_name": model_name,
        "xyz": stage_center,
        "rpy": [0, 0, 0]
    })
    
    return objects

def write_world_config(scenario, world_file_path, output_folder, output_world_file_name):
    # Read the world
    world_config = read_yaml(world_file_path)

    # Make Output and Model directory
    os.makedirs(output_folder, exist_ok=True)
    output_models_folder = os.path.join(output_folder, "models")
    os.makedirs(output_models_folder, exist_ok=True)

    print('scenario: ', scenario)

    # Add obstacles as objects
    if "objects" not in world_config:
      world_config["objects"] = []

    if "stage2" in scenario:
      window_objs = generate_as2_windows_config(scenario, output_models_folder)
      world_config["objects"].extend(window_objs)

    if "stage3" in scenario:
      forest_objs = generate_as2_forest_config(scenario, output_models_folder)
      world_config["objects"].extend(forest_objs)
    
    if "stage4" in scenario:
      dynamic_objs = generate_as2_dynamic_objects_config(scenario, output_models_folder)
      world_config["objects"].extend(dynamic_objs)

    if "stage_bridge" in scenario:
      print('reached section bridge if statement.')
      bridge_objs = generate_as2_bridge_config(scenario, output_models_folder)
      print(bridge_objs)
      world_config["objects"].extend(bridge_objs)
      
    

    floor_objs = generate_as2_floor_tiles_config(scenario, output_models_folder)
    world_config["objects"].extend(floor_objs)

    with open(os.path.join(output_folder, output_world_file_name), 'w') as file:
        yaml.dump(world_config, file)

def main():
    parser = argparse.ArgumentParser(description="Generate JSON world configuration from YAML scenario.")
    parser.add_argument('input_scenario', type=str, help="Path to the input YAML scenario file.")
    parser.add_argument('--output_folder', "-o", type=str, 
                        default=os.path.join("config_sim", "world"),
                        help="Folder to the output YAML world configuration file and generated models")
    parser.add_argument('--world_template', "-w", type=str, 
                        default=os.path.join("config_sim", "config", "world.yaml"),
                        help="Path to the input World YAML template file.")
    parser.add_argument("--output_world_file_name", "-f", default="world.yaml", help="Output file")

    args = parser.parse_args()

    print(f"Processing scenario {args.input_scenario}")
    scenario = read_yaml(args.input_scenario)
    print(scenario)
    print("World template", args.world_template)
    print("Output folder", args.output_folder)
    write_world_config(scenario, args.world_template, args.output_folder, args.output_world_file_name)
    print(f"Scenario from {args.input_scenario} with template {args.world_template} has world configuration written to {os.path.join(args.output_folder, args.output_world_file_name)}")

if __name__ == "__main__":
    main()