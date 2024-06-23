# Autonomous Driving using ROS for water vehicles

This repository contains the necessary submodules and instructions to set up autonomous driving using TurtleBot3 with ROS2 Humble. The repository includes the navigation packages for TurtleBot3 and other necessary dependencies.

## Requirements

- **Ubuntu 22.04 (Jammy Jellyfish)**
- **ROS 2 Humble Hawksbill**

## Setup

### 1. Clone the Repository

First, clone this repository along with its submodules:

```bash
git clone --recurse-submodules https://github.com/BaptisteAmare/autonomousDriving.git
cd autonomousDriving
```

### 2. Build the Workspace

Make sure you have ROS2 Humble installed on your system. Follow the [ROS2 installation guide](https://docs.ros.org/en/humble/Installation.html) if you haven't installed it yet.

Build the workspace:

```bash
colcon build
```

### 3. Source the Setup Files

Before running the ROS2 commands, source the setup files:

```bash
source ~/turtlebot3_ws/install/setup.bash
source /opt/ros/humble/setup.bash
```

If you don’t want to have to source the setup file every time you open a new shell, then you can add the command to the shell startup script:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

After adding these lines to the `.bashrc` file, source the file:

```bash
source ~/.bashrc
```

### 4. Install Gazebo Simulator

Install Gazebo11 and its associated ROS 2 meta-packages by typing the following command in a shell:

```bash
sudo apt install gazebo11
sudo apt install ros-humble-gazebo-ros-pkgs
```

### 5. Install ROS 2 Dependent Packages

#### 5.1. Cartographer

Cartographer is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations.

Type the following command in a shell to install the ROS 2 Cartographer packages:

```bash
sudo apt install ros-humble-cartographer 
sudo apt install ros-humble-cartographer-ros
```

#### 5.2. Navigation Stack for ROS 2

The ROS 2 Navigation Stack is a set of packages that helps the robot move from start position to the goal position.

Type the following command in a shell to install the ROS 2 Navigation Stack packages:

```bash
sudo apt install ros-humble-navigation2 
sudo apt install ros-humble-nav2-bringup
```

## Usage

### 1. Running the Turtlebot3 Gazebo Simulation

Navigate inside the workspace (`turtlebot3_ws`), and source the workspace:

```bash
source ~/turtlebot3_ws/install/setup.bash
```

TurtleBot3 has three models, burger, waffle, and waffle_pi, so you have to set which model to use before using. To do this on Ubuntu, we specify the model to be used with the export command. Type the following commands in a shell to add `TURTLEBOT3_MODEL` as burger to the `.bashrc` file. After that, source the `.bashrc` file:

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

#### 1.1. Launching Turtlebot3 Simulation World in Gazebo

Open a new shell window and source the turtlebot3 workspace:

```bash
source ~/turtlebot3_ws/install/setup.bash
```

Launch the Turtlebot3 robot in the TurtleBot3 World by typing the following command:

```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

You can change the simulation world by launching other simulation launch files:

- **TurtleBot3 World**:
  ```bash
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  ```
- **TurtleBot3 House**:
  ```bash
  ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
  ```

### 2. Running Turtlebot3 Teleoperation Node

Open a new shell window and source the turtlebot3 workspace:

```bash
source ~/turtlebot3_ws/install/setup.bash
```

Run the teleoperation node with the following command to teleoperate the TurtleBot3 with the keyboard:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### 3. Launch the Cartographer for SLAM

To start the Cartographer for simultaneous localization and mapping (SLAM) with simulation time enabled:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### 4. Save the Map

To save the generated map, run:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 5. Launch Navigation2

To launch the Navigation2 stack with a specific map and simulation time enabled:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=maps/stage4.yaml
```

To see all the arguments available for the Navigation2 launch file:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py --show-args
```

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