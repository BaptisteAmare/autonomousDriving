# Autonomous Driving for boats

This repository contains the necessary submodules and instructions to set up autonomous driving using TurtleBot3 with ROS2 Humble. The repository includes the navigation packages for TurtleBot3 and other necessary dependencies.

## Setup

### Clone the Repository

First, clone this repository along with its submodules:

```bash
git clone --recurse-submodules https://github.com/BaptisteAmare/autonomousDriving.git
cd autonomousDriving
```

### Build the Workspace

Make sure you have ROS2 Humble installed on your system. Follow the [ROS2 installation guide](https://docs.ros.org/en/humble/Installation.html) if you haven't installed it yet.

Build the workspace:

```bash
colcon build
```

### Source the Setup Files

Before running the ROS2 commands, source the setup files:

```bash
source ~/autonomousDriving/install/setup.bash
source /opt/ros/humble/setup.bash
```

## Usage

### Launch the TurtleBot3 World in Gazebo

To launch the TurtleBot3 world in the Gazebo simulator, use the following command:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Launch the Cartographer for SLAM

To start the Cartographer for simultaneous localization and mapping (SLAM) with simulation time enabled:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### Save the Map

To save the generated map, run:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Teleoperate the TurtleBot3

To manually control the TurtleBot3 using your keyboard:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### Launch Navigation2

To launch the Navigation2 stack with a specific map and simulation time enabled:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=maps/stage4.yaml
```

To see all the arguments available for the Navigation2 launch file:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py --show-args
```

### Launch Navigation2 with Full Path to Map

To launch the Navigation2 stack with a full path to the map file and simulation time enabled:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=src/maps/stage4/stage4.yaml
```

## Additional Information

For more detailed information on each package and its usage, please refer to the official documentation of each submodule:

- [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [TurtleBot3 Messages](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)

## Troubleshooting

If you encounter any issues, ensure that all dependencies are correctly installed and sourced. Refer to the official ROS2 and TurtleBot3 documentation for additional support.

## Contribution

Feel free to open issues or submit pull requests if you have any suggestions or improvements.

## License

This project is licensed under the [Apache 2.0 License](LICENSE).
```
