---
title: Robot Control and Programming
sidebar_position: 5
---

# Robot Control and Programming

Welcome to the exciting world of robot control and programming! In this chapter, we'll explore the sophisticated software systems that bring robots to life. You'll learn how to program robots to move, balance, interact with their environment, and perform complex tasks. This is where hardware meets intelligence to create truly capable robotic systems.

## What You Will Learn

In this chapter, you'll:

- Master fundamental robot control concepts and algorithms
- Learn to implement PID controllers for precise motor control
- Understand kinematics and how to calculate robot movements
- Program motion planning algorithms for navigation
- Implement balance control for bipedal robots
- Work with ROS2 control frameworks
- Create coordinated multi-joint movements
- Develop robot behaviors using state machines

By the end of this chapter, you'll have the programming skills needed to control complex robotic systems and make them perform meaningful tasks.

## Control Systems Fundamentals

Robot control is the process of commanding a robot's actuators to achieve desired movements or behaviors. Control systems can be classified into several categories:

### Open-Loop vs Closed-Loop Control
- **Open-loop**: Commands sent without feedback (like a timer-based system)
- **Closed-loop**: Uses sensor feedback to adjust commands (like a thermostat)

Closed-loop control is essential for precise robot operation, as it compensates for disturbances and model inaccuracies.

## PID Control

PID (Proportional-Integral-Derivative) control is the most common control algorithm in robotics. It adjusts the control signal based on three components:

- **Proportional (P)**: Responds to current error
- **Integral (I)**: Responds to accumulated past error
- **Derivative (D)**: Responds to rate of error change

```python
class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.dt = dt  # Time step

        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * self.dt
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative

        # Store error for next iteration
        self.prev_error = error

        # Calculate output
        output = p_term + i_term + d_term
        return output
```

### Tuning PID Parameters
- **Kp too high**: Oscillations and instability
- **Kp too low**: Slow response and poor tracking
- **Ki too high**: Overshoot and oscillations
- **Ki too low**: Steady-state error
- **Kd too high**: Noise amplification
- **Kd too low**: Insufficient damping

## Kinematics in Robot Control

Kinematics is the study of motion without considering forces. For robots, we distinguish between:

### Forward Kinematics
Given joint angles, calculate the end-effector position. This is used for understanding where the robot's tools are in space.

```python
import numpy as np

def forward_kinematics_2dof(theta1, theta2, l1, l2):
    """
    Calculate end-effector position for 2-DOF planar manipulator
    """
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return x, y
```

### Inverse Kinematics
Given desired end-effector position, calculate required joint angles. This is crucial for reaching desired positions.

```python
def inverse_kinematics_2dof(x, y, l1, l2):
    """
    Calculate joint angles for 2-DOF planar manipulator to reach (x, y)
    """
    # Distance from origin to target
    r = np.sqrt(x**2 + y**2)

    # Check if target is reachable
    if r > l1 + l2:
        raise ValueError("Target is outside workspace")

    # Calculate theta2
    cos_theta2 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = np.arccos(np.clip(cos_theta2, -1, 1))

    # Calculate theta1
    k1 = l1 + l2 * np.cos(theta2)
    k2 = l2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return theta1, theta2
```

## Motion Planning

Motion planning algorithms help robots navigate from start to goal while avoiding obstacles.

### Path Planning vs Trajectory Planning
- **Path planning**: Find collision-free path through space
- **Trajectory planning**: Add timing and velocity profiles to the path

### Common Algorithms
- **A* Algorithm**: Grid-based optimal pathfinding
- **RRT (Rapidly-exploring Random Trees)**: Sampling-based planning
- **Potential Fields**: Gradient-based navigation

```python
import numpy as np
import matplotlib.pyplot as plt

def simple_trajectory_planning(start, goal, duration, dt):
    """
    Generate smooth trajectory from start to goal
    using trapezoidal velocity profile
    """
    # Calculate required velocity
    total_distance = goal - start
    max_velocity = total_distance / (duration * 0.8)  # 80% acceleration/deceleration

    # Time parameters
    t = np.arange(0, duration, dt)

    # Trapezoidal profile parameters
    acceleration_time = duration * 0.2
    constant_time = duration * 0.6
    deceleration_time = duration * 0.2

    positions = []
    velocities = []

    for time in t:
        if time < acceleration_time:
            # Acceleration phase
            vel = max_velocity * (time / acceleration_time)
            pos = start + 0.5 * max_velocity * (time**2) / acceleration_time
        elif time < acceleration_time + constant_time:
            # Constant velocity phase
            vel = max_velocity
            elapsed = time - acceleration_time
            pos = start + 0.5 * max_velocity * acceleration_time + max_velocity * elapsed
        else:
            # Deceleration phase
            elapsed = time - (acceleration_time + constant_time)
            vel = max_velocity * (1 - elapsed / deceleration_time)
            pos = (start + 0.5 * max_velocity * acceleration_time +
                   max_velocity * constant_time +
                   max_velocity * elapsed - 0.5 * max_velocity * (elapsed**2) / deceleration_time)

        positions.append(pos)
        velocities.append(vel)

    return np.array(positions), np.array(velocities)
```

## Balance Control for Humanoid Robots

Balance control is critical for bipedal robots. The most common approach is the Linear Inverted Pendulum Model (LIPM).

### Zero Moment Point (ZMP)
ZMP is a point where the net moment of the ground reaction forces equals zero. For stable walking, the ZMP must remain within the support polygon (foot area).

```python
class BalanceController:
    def __init__(self, robot_height, kp=10.0, ki=1.0, kd=2.0):
        self.robot_height = robot_height
        self.g = 9.81  # Gravity constant
        self.omega = np.sqrt(self.g / robot_height)

        # PID gains for ZMP control
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_error_x = 0
        self.prev_error_y = 0
        self.integral_x = 0
        self.integral_y = 0

    def compute_balance_correction(self, current_zmp_x, current_zmp_y,
                                 desired_zmp_x, desired_zmp_y, dt):
        """
        Compute balance correction based on ZMP error
        """
        # Calculate errors
        error_x = desired_zmp_x - current_zmp_x
        error_y = desired_zmp_y - current_zmp_y

        # X-axis PID control
        self.integral_x += error_x * dt
        derivative_x = (error_x - self.prev_error_x) / dt if dt > 0 else 0

        correction_x = (self.kp * error_x +
                       self.ki * self.integral_x +
                       self.kd * derivative_x)

        # Y-axis PID control
        self.integral_y += error_y * dt
        derivative_y = (error_y - self.prev_error_y) / dt if dt > 0 else 0

        correction_y = (self.kp * error_y +
                       self.ki * self.integral_y +
                       self.kd * derivative_y)

        # Update previous errors
        self.prev_error_x = error_x
        self.prev_error_y = error_y

        return correction_x, correction_y
```

## ROS2 Control Framework

ROS2 provides several packages for robot control:

### ros2_control
The standard framework for robot control in ROS2. It includes:

- **Controller Manager**: Manages different controllers
- **Hardware Interface**: Abstracts hardware communication
- **Controllers**: Joint trajectory controllers, position controllers, etc.

### Example Controller Configuration:
```yaml
# controller_manager.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

position_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

## State Machines for Robot Behaviors

State machines are excellent for organizing complex robot behaviors:

```python
from enum import Enum
import time

class RobotState(Enum):
    IDLE = 1
    WALKING = 2
    GRASPING = 3
    TALKING = 4
    EMERGENCY_STOP = 5

class RobotStateMachine:
    def __init__(self):
        self.current_state = RobotState.IDLE
        self.start_time = time.time()

    def update(self, sensor_data):
        """Update robot state based on sensors and current state"""

        # Check for emergency conditions first
        if sensor_data.get('emergency_stop', False):
            self.current_state = RobotState.EMERGENCY_STOP
            return

        # State-specific logic
        if self.current_state == RobotState.IDLE:
            if sensor_data.get('goal_received', False):
                self.current_state = RobotState.WALKING
                self.start_time = time.time()

        elif self.current_state == RobotState.WALKING:
            if sensor_data.get('goal_reached', False):
                self.current_state = RobotState.IDLE
            elif sensor_data.get('object_detected', False):
                self.current_state = RobotState.GRASPING

        elif self.current_state == RobotState.GRASPING:
            if sensor_data.get('grasp_complete', False):
                self.current_state = RobotState.IDLE

    def execute_current_behavior(self):
        """Execute behavior based on current state"""
        if self.current_state == RobotState.IDLE:
            return self.idle_behavior()
        elif self.current_state == RobotState.WALKING:
            return self.walking_behavior()
        elif self.current_state == RobotState.GRASPING:
            return self.grasping_behavior()
        elif self.current_state == RobotState.TALKING:
            return self.talking_behavior()
        elif self.current_state == RobotState.EMERGENCY_STOP:
            return self.emergency_stop_behavior()

    def idle_behavior(self):
        return "Robot is idle, waiting for commands"

    def walking_behavior(self):
        return "Robot is walking to goal position"

    def grasping_behavior(self):
        return "Robot is grasping object"

    def talking_behavior(self):
        return "Robot is engaging in conversation"

    def emergency_stop_behavior(self):
        return "EMERGENCY STOP - Robot halted"
```

## Whole-Body Control

For complex humanoid robots, whole-body control coordinates multiple subsystems:

- **Task prioritization**: High-priority tasks (balance) take precedence
- **Null-space projection**: Lower-priority tasks executed in null space
- **Constraint handling**: Joint limits, torque limits, etc.

## Mini Lab / Exercise

**Implementing a Simple Robot Controller**

In this exercise, you'll implement a basic robot controller:

### Prerequisites:
- Python with numpy and matplotlib
- Basic understanding of control systems

### Steps:
1. Implement a PID controller for a single joint
2. Test the controller with different parameters
3. Implement inverse kinematics for a simple 2-DOF arm
4. Create a trajectory generator
5. Combine all components into a simple control system

### Sample Code Structure:
```python
# 1. Create a simple robot simulation
class SimpleRobot:
    def __init__(self):
        self.position = 0
        self.velocity = 0
        self.joint_angle = 0

    def update(self, torque, dt):
        # Simple physics simulation
        acceleration = torque / 0.1  # Assume mass = 0.1
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

# 2. Implement your controller
# 3. Test with different trajectories
# 4. Analyze performance

# Questions to explore:
# - How do PID parameters affect performance?
# - What happens with different trajectory profiles?
# - How does noise affect control performance?
```

### Questions to Consider:
- How does the choice of control frequency affect performance?
- What are the trade-offs between different control strategies?
- How would you adapt this controller for a multi-joint robot?
- What safety considerations are important in robot control?

## Next Chapter Preview

In the next chapter, "AI and Perception for Robotics," we'll explore how robots perceive and understand their environment using artificial intelligence. You'll learn about computer vision, sensor fusion, machine learning for robotics, and how robots make intelligent decisions based on sensory input.

We'll dive into practical applications like object recognition, navigation, and human-robot interaction using AI techniques. You'll implement real AI algorithms that enable robots to see, understand, and respond to their environment intelligently.

This chapter will bridge the gap between robot control and artificial intelligence, showing you how to create truly intelligent robotic systems.

Get ready to make your robots smarter!