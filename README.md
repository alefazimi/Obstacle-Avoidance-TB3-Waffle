# TurtleBot3 Obstacle Avoidance with SLAM

An autonomous navigation system for TurtleBot3 that explores unknown environments while avoiding obstacles and building a map in real-time.

## Overview

This project implements a reactive obstacle avoidance behavior for TurtleBot3 robots using ROS2. The robot navigates autonomously by processing laser scan data, making real-time navigation decisions, and simultaneously creating a map of the environment using SLAM Toolbox.

## Features

- **Real-time obstacle detection** using 360° laser scanner
- **Smart navigation** - robot turns toward open space when obstacles detected
- **Simultaneous mapping** with SLAM Toolbox integration
- **Automatic recovery** - switches direction if stuck
- **Fully configurable** parameters via YAML file
- **Timed launch system** - coordinated startup of all components
- **RViz2 visualization** - live view of robot state and map

## How It Works

The robot divides its field of view into three sectors:
- **Front Center** (-20° to +20°): Detects obstacles directly ahead
- **Front Left** (+20° to +80°): Checks left side clearance  
- **Front Right** (-80° to -20°): Checks right side clearance

**Navigation Logic:**
1. If front is clear → Move forward at 0.15 m/s
2. If obstacle detected within 0.6m → Stop and turn toward the side with more space
3. If turning for >3 seconds → Try opposite direction
4. Repeat

## Requirements

- ROS2 (Humble/Foxy or later)
- TurtleBot3 packages
- Gazebo simulator
- SLAM Toolbox
- RViz2

## Installation

```bash
# Clone this repository into your ROS2 workspace
cd ~/ros2_ws/src
git clone <your-repo-url>

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select tb3_obstacle_avoidance

# Source the workspace
source install/setup.bash
```

## Usage

Launch the complete simulation (Gazebo + SLAM + Obstacle Avoidance + RViz):

```bash
ros2 launch tb3_obstacle_avoidance sim_tb3_avoidance.launch.py
```

**Optional arguments:**
```bash
ros2 launch tb3_obstacle_avoidance sim_tb3_avoidance.launch.py model:=waffle
```

**What happens:**
- t=0s: Gazebo launches with TurtleBot3 World
- t=5s: RViz2 opens with mapping visualization
- t=15s: Robot starts moving autonomously

## Configuration

Edit `config/obstacle_avoidance.yaml` to adjust parameters:

```yaml
obstacle_avoidance:
  ros__parameters:
    # Speed settings
    linear_speed: 0.15        # Forward speed (m/s)
    angular_speed: 0.6        # Turning speed (rad/s)
    
    # Detection
    obstacle_distance: 0.6    # Trigger distance for avoidance (m)
    
    # Timing
    startup_delay_sec: 1.0    # Delay before movement starts
```

## Project Structure

```
tb3_obstacle_avoidance/
├── src/
│   └── obstacle_avoidance_node.cpp    # Main navigation logic
├── launch/
│   └── sim_tb3_avoidance.launch.py    # Launch file
├── config/
│   ├── obstacle_avoidance.yaml        # Parameters
│   └── rviz/
│       └── tb3_mapping.rviz           # RViz config
├── CMakeLists.txt
└── package.xml
```

## Key Concepts Demonstrated

- ROS2 node development in C++
- Laser scan data processing
- Reactive navigation strategies
- SLAM integration for mapping
- Parameter-based configuration
- Multi-node launch coordination

## Limitations

- Purely reactive behavior (no path planning)
- May get stuck in complex obstacle configurations
- No goal-directed navigation
- Assumes static environment

