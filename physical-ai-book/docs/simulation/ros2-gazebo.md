---
title: ROS2 and Gazebo Simulation
sidebar_position: 3
---

# ROS2 and Gazebo Simulation

Welcome to the world of robot simulation! In this chapter, we'll explore ROS2 (Robot Operating System 2) and Gazebo, the two essential tools that form the backbone of modern robotics development. These tools allow you to develop, test, and debug robot software in a safe virtual environment before deploying to real hardware.

## What You Will Learn

In this chapter, you'll:

- Understand the fundamentals of ROS2 and its role in robotics
- Learn to set up and configure a ROS2 development environment
- Master the concepts of nodes, topics, services, and actions
- Create your first ROS2 package and nodes
- Simulate robots in the Gazebo physics engine
- Work with robot models in URDF (Unified Robot Description Format)
- Control simulated robots using ROS2 commands

By the end of this chapter, you'll have a complete ROS2 development environment and be able to simulate and control virtual robots.

## What is ROS2?

ROS2 (Robot Operating System 2) is not an operating system but rather a flexible framework for writing robot software. It provides libraries and tools to help software developers create robot applications. ROS2 is the successor to ROS1, with improvements in real-time performance, security, and scalability.

Key features of ROS2 include:

- **Distributed computing**: Multiple processes can communicate seamlessly
- **Hardware abstraction**: Write code that works across different hardware
- **Reusable components**: Leverage existing packages and tools
- **Language support**: C++, Python, and other languages
- **Real-time support**: Better timing guarantees for critical applications

## ROS2 Architecture Concepts

### Nodes
A node is a process that performs computation. In ROS2, nodes are written in various programming languages and work together to form a complete robotic system. Each node can publish or subscribe to messages, provide services, or execute actions.

### Topics and Messages
Topics are named buses over which nodes exchange messages. Messages are the data packets sent between nodes. Multiple nodes can publish to the same topic, and multiple nodes can subscribe to the same topic.

### Services
Services provide a request/response communication pattern. A client sends a request to a service server, which processes the request and returns a response.

### Actions
Actions are used for long-running tasks that may take time to complete. They provide feedback during execution and can be preempted if needed.

## Setting Up Your ROS2 Environment

First, you'll need to install ROS2. The current recommended distribution is ROS2 Humble Hawksbill (Ubuntu 22.04) or ROS2 Iron Irwini (Ubuntu 24.04). For Windows users, WSL2 with Ubuntu is recommended.

```bash
# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
colcon build
source install/setup.bash
```

## Creating Your First ROS2 Package

Let's create a simple ROS2 package for robot simulation:

```bash
# Navigate to your workspace source directory
cd ~/ros2_ws/src

# Create a new package
ros2 pkg create --build-type ament_python robot_simulation_tutorial --dependencies rclpy std_msgs geometry_msgs

# Navigate to the package
cd robot_simulation_tutorial
```

Now let's create a simple publisher node that publishes velocity commands:

```python
#!/usr/bin/env python3
# File: robot_simulation_tutorial/robot_simulation_tutorial/velocity_publisher.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0  # Move forward at 1 m/s
        msg.angular.z = 0.5  # Rotate at 0.5 rad/s
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: linear={msg.linear.x}, angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Introduction to Gazebo Simulation

Gazebo is a 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It's widely used in robotics research and development.

Key features of Gazebo include:
- **Realistic physics**: Accurate simulation of rigid body dynamics
- **Sensors**: Cameras, LIDAR, IMUs, GPS, and more
- **Environment modeling**: Complex indoor and outdoor scenes
- **ROS2 integration**: Seamless integration with ROS2 through gazebo_ros_pkgs

## Working with Robot Models (URDF)

URDF (Unified Robot Description Format) is an XML format for representing robot models. It describes the robot's physical and visual properties.

Here's a simple URDF for a differential drive robot:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

## Launching Your First Simulation

To run a simulation, you'll typically use launch files. Here's a simple launch file:

```python
# File: robot_simulation_tutorial/launch/simple_robot.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'simple_robot'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True}]
        )
    ])
```

## Running the Simulation

To run your simulation:

```bash
# Build your workspace
cd ~/ros2_ws
colcon build --packages-select robot_simulation_tutorial

# Source the setup
source install/setup.bash

# Launch the simulation
ros2 launch robot_simulation_tutorial simple_robot.launch.py
```

## Controlling Your Robot

Once the simulation is running, you can control your robot using ROS2 commands:

```bash
# Send velocity commands to move the robot
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```

## Mini Lab / Exercise

**Creating and Simulating a Simple Robot**

In this exercise, you'll create a simple robot model and simulate it in Gazebo:

### Prerequisites:
- ROS2 Humble installed
- Gazebo installed
- Basic Python knowledge

### Steps:
1. Create a new ROS2 package as shown above
2. Create the URDF file for a simple robot with a base and two wheels
3. Create a launch file to spawn your robot in Gazebo
4. Build and run the simulation
5. Control the robot using ROS2 commands

### Questions to Explore:
- How does changing the wheel radius affect the robot's movement?
- What happens when you change the mass properties in the URDF?
- How can you add sensors to your robot model?
- What other joint types exist besides continuous joints?

## Next Chapter Preview

In the next chapter, "Humanoid Robot Design," we'll dive into the specific challenges and considerations involved in designing humanoid robots. You'll learn about the unique mechanical, control, and computational requirements of bipedal robots.

We'll explore the anatomy of humanoid robots, including their joint configurations, degrees of freedom, and balance control systems. You'll also learn about existing humanoid platforms like NAO, Pepper, and Atlas, and understand the design decisions behind their configurations.

This chapter will prepare you for the more complex programming and control challenges that humanoid robots present, building on the simulation foundation we've established here.

Get ready to explore the fascinating world of humanoid robot design!