# TurtleBot3 Agent with ROS2 Navigation

This repository contains the TurtleBot3 Agent code for interacting with a TurtleBot3 robot using ROS2 and the Nav2 stack. Follow the instructions below to set up and run the agent alongside the TurtleBot3 simulation and navigation stack.

---

## Prerequisites

1. **ROS2 Humble**: Ensure ROS2 Humble is installed on your system.
2. **TurtleBot3 Packages**: Install the TurtleBot3 packages and dependencies.
3. **Gazebo**: Install Gazebo for simulation.
4. **Map File**: Ensure you have a map file (e.g., `/home/rahul/map.yaml`) for navigation.

---

## Setup Instructions

### Step 1: Clone and Build the Workspace

1. Create a ROS2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Clone this repository into the `src` directory:
   ```bash
   git clone <repository_url> turtlebot3_agent
   ```

3. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select turtlebot3_agent
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---

### Step 2: Run the TurtleBot3 Simulation

1. Source the ROS2 and TurtleBot3 workspaces:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/turtlebot3_ws/install/setup.bash
   export TURTLEBOT3_MODEL=burger
   ```

2. Launch the TurtleBot3 Gazebo simulation:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

---

### Step 3: Run the Navigation Stack

1. In a new terminal, source the ROS2 and TurtleBot3 workspaces:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/turtlebot3_ws/install/setup.bash
   export TURTLEBOT3_MODEL=burger
   ```

2. Launch the navigation stack with your map:
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/rahul/map.yaml
   ```

---

### Step 4: Run the TurtleBot3 Agent

1. In another new terminal, source the ROS2 workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

2. Run the agent node:
   ```bash
   ros2 run turtlebot3_agent agent_node
   ```

---

## Execution Order

1. **Terminal 1**: Run the Gazebo simulation.
2. **Terminal 2**: Run the navigation stack.
3. **Terminal 3**: Run the agent node.

run one after other
Ensure all three terminals are active and running simultaneously.

---

## Notes

- Replace `<repository_url>` with the actual URL of this repository.
- Update the map file path (`/home/rahul/map.yaml`) to the location of your map file.
- The agent interacts with the TurtleBot3 robot via the ROS2 Navigation stack. Ensure the navigation stack is fully operational before starting the agent.

Enjoy exploring with your TurtleBot3!