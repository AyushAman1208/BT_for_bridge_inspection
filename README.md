# COMP0240 Multi-Drone Challenge CW2

## Installation

To install this project, create your ros workspace and clone the following into the source repo:

```bash
mkdir -p bridge_inspection_mission/src
cd bridge_inspection_mission/src
Install aerostack following this documentation https://aerostack2.github.io/_00_getting_started/index.html
git clone https://github.com/AyushAman1208/BT_for_bridge_inspection.git
```


```

Please go to the root folder of the project and build it:

```bash
cd BT_for_bridge_inspection
colcon build
```




## Execution

### 1. Launch aerostack2 nodes for each drone
To launch aerostack2 nodes for each drone, execute once the following command:

```bash
./launch_as2.bash -s scenario_bridge.yaml
```
> **NOTE:** This project was built on top of a coursework repo and therefore contains multiple scenarios and files which are not relevant for this project.


### 2. Launch aerostack2 nodes for the ground station
To launch aerostack2 nodes for the ground station, execute once the following command:

```bash
./launch_ground_station.bash
```

The flags for the components launcher are:


### 3. Launch a mission
There are several missions that can be executed:

- **AS2 Multi Drone**: 
  
  In terminal 1
  ```bash
  ./launch_as2.bash
  ```

  In a different terminal 2
  ```bash
  ./launch_ground_station.bash
  ```

  And then in one of your ground station terminals (or separately in a different terminal)
  ```bash
  python3 <Path of the file you want to run> 
  ```





### 4. End the execution

If you are using tmux, you can end the execution with the following command:

```bash
./stop.bash
```

You can force the end of all tmux sessions with the command:
```bash
tmux kill-server
```

If you are using gnome-terminal, you can end the execution by closing the terminal.

> Note sometimes you may find gazebo stays running for some reason. It is recommended that you install hte `htop` utility. Running htop use F4 to search for gazebo. Select the running gazebo process and press F9. Then select `SIGKILL` and that will kill it. 

## Running Notes:

If you are running on ubuntu, depending on your system you may see a few different things going wrong:

1. Gazebo struggles to load up assets - this is likely either a ROS2 launch problem or a not enough CPU problem leading to race conditions. Launch with fewer drones. (Maybe even look into using the AS2 Multicopter rather than gazebo)
2. Aerostack2/ROS2 struggles with lots of drones - this is likely a not enough CPU problem


One thing that might help is forcing your terminal to use the NVIDIA GPU. It seems (depending on the laptop/desktop setup) that sometimes the terminal window will not run gazebo in gpu mode. To test this run this system, and in a separate terminal check `nvidia-smi`. If gazebo is using the GPU it will show there. If its not, you can force it to use the discrete GPU by adding the following lines to your `~/.bashrc` and restart your terminal, it should enable the GPU:

```
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

## Developers guide

**Slightly out of date**

All projects in aerostack2 are structured in the same way. The project is divided into the following directories:

- **tmuxinator**: Contains the tmuxinator launch file, which is used to launch all aerostack2 nodes.
  - **aerostack2.yaml**: Tmuxinator launch file for each drone. The list of nodes to be launched is defined here.
  - **ground_station.yaml**: Tmuxinator launch file for the ground station. The list of nodes to be launched is defined here.
- **config**: Contains the configuration files for the launchers of the nodes in the drones.
- **config_ground_station**: Contains the configuration files for the launchers of the nodes in the ground station.
- **launch_as2.bash**: Script to launch nodes defined in *tmuxinator/aerostack2.yaml*.
- **launch_ground_station.bash**: Script to launch nodes defined in *tmuxinator/ground_station.yaml*.
- **mission_\*.py**: Differents python mission files that can be executed.
- **stop_tmuxinator_as2.bash**: Script to stop all nodes launched by *launch_as2.bash*.
- **stop_tmuxinator_ground_station.bash**: Script to stop all nodes launched by *launch_ground_station.bash*.
- **stop_tmuxinator.bash**: Script to stop all nodes launched by *launch_as2.bash* and *launch_ground_station.bash*.
- **rosbag/record_rosbag.bash**: Script to record a rosbag. Can be modified to record only the topics that are needed.
- **trees\***: Contains the behavior trees that can be executed. They can be selected in the *aerostack2.yaml* file.
- **utils**: Contains utils scripts for launchers.

Both python and bash scripts have a help message that can be displayed by running the script with the `-h` option. For example, `./launch_as2.bash -h` will display the help message for the `launch_as2.bash` script.

**Note**: For knowing all parameters for each launch, you can execute the following command:

```bash
ros2 launch my_package my_launch.py -s
```

Also, you can see them in the default config file of the package, in the *config* folder. If you want to modify the default parameters, you can add the parameter to the config file.
