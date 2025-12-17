# Chapter 8 Lab: Balance Control for Humanoid Robots

## Objective

The goal of this lab is to implement and test balance control algorithms for humanoid robots, focusing on stability, feedback control, and dynamic balance maintenance. You will learn to design PID controllers, implement balance algorithms, and test stability in simulated humanoid robot environments.

## Learning Outcomes

By the end of this lab, you should be able to:

- Implement PID controllers for robot balance
- Understand and apply Zero Moment Point (ZMP) theory
- Design feedback control systems for stability
- Simulate and test balance control in humanoid robots
- Analyze stability margins and system performance
- Handle disturbances and maintain balance recovery

## Prerequisites

- ROS2 Humble Hawksbill installed
- Gazebo simulation environment
- Basic understanding of control theory
- Python programming skills
- Knowledge of kinematics from Chapter 5

## Materials Needed

- Computer with ROS2 and Gazebo installed
- Terminal access
- Text editor or IDE
- Basic understanding of linear algebra and differential equations

## Background

Balance control is critical for humanoid robots, as they are inherently unstable due to their bipedal design. This lab explores various control strategies to maintain balance, including PID control, ZMP-based control, and whole-body control approaches.

## Lab Procedure

### Part 1: Setting Up the Environment (15 minutes)

1. **Create a new ROS2 package for this lab:**
   ```bash
   cd ~/ros2_labs/src
   ros2 pkg create --build-type ament_python lab_balance_control --dependencies rclpy geometry_msgs sensor_msgs std_msgs builtin_interfaces
   ```

2. **Navigate to the package directory:**
   ```bash
   cd lab_balance_control
   ```

3. **Create the main script directory:**
   ```bash
   mkdir -p lab_balance_control
   ```

### Part 2: Implementing PID Controllers (45 minutes)

Create a comprehensive PID controller implementation for balance control:

```python
#!/usr/bin/env python3
import numpy as np
import math
from dataclasses import dataclass
from typing import Tuple

@dataclass
class PIDParams:
    kp: float = 1.0
    ki: float = 0.1
    kd: float = 0.05
    min_output: float = -1.0
    max_output: float = 1.0

class PIDController:
    def __init__(self, params: PIDParams):
        self.kp = params.kp
        self.ki = params.ki
        self.kd = params.kd
        self.min_output = params.min_output
        self.max_output = params.max_output

        # Internal state
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None

        # Anti-windup parameters
        self.windup_guard = 1.0

    def update(self, setpoint: float, measurement: float, dt: float = None) -> float:
        """
        Update PID controller with new measurement
        """
        current_time = time.time()

        if dt is None:
            if self.previous_time is not None:
                dt = current_time - self.previous_time
            else:
                dt = 0.01  # Default time step
            self.previous_time = current_time

        # Calculate error
        error = setpoint - measurement

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * dt
        # Anti-windup
        if self.integral > self.windup_guard:
            self.integral = self.windup_guard
        elif self.integral < -self.windup_guard:
            self.integral = -self.windup_guard
        i_term = self.ki * self.integral

        # Derivative term
        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative

        # Calculate output
        output = p_term + i_term + d_term

        # Clamp output
        output = max(self.min_output, min(self.max_output, output))

        # Update for next iteration
        self.previous_error = error

        return output

    def reset(self):
        """Reset the PID controller internal state"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None

class BalancePIDController:
    def __init__(self):
        # PID controllers for different balance aspects
        self.pitch_controller = PIDController(PIDParams(kp=2.0, ki=0.1, kd=0.3))
        self.roll_controller = PIDController(PIDParams(kp=2.0, ki=0.1, kd=0.3))
        self.zmp_controller = PIDController(PIDParams(kp=1.5, ki=0.05, kd=0.2))

    def calculate_balance_torques(self, current_state: dict) -> dict:
        """
        Calculate balance torques based on current state
        current_state should contain:
        - pitch_angle: current pitch angle
        - roll_angle: current roll angle
        - pitch_rate: current pitch angular velocity
        - roll_rate: current roll angular velocity
        - zmp_x, zmp_y: current ZMP position
        - target_zmp_x, target_zmp_y: desired ZMP position
        """
        dt = 0.01  # 100Hz control rate

        # Calculate balance torques for pitch and roll
        pitch_torque = self.pitch_controller.update(
            setpoint=0.0,  # Target pitch angle
            measurement=current_state['pitch_angle'],
            dt=dt
        )

        roll_torque = self.roll_controller.update(
            setpoint=0.0,  # Target roll angle
            measurement=current_state['roll_angle'],
            dt=dt
        )

        # Calculate ZMP-based adjustments
        zmp_x_adjustment = self.zmp_controller.update(
            setpoint=current_state['target_zmp_x'],
            measurement=current_state['zmp_x'],
            dt=dt
        )

        zmp_y_adjustment = self.zmp_controller.update(
            setpoint=current_state['target_zmp_y'],
            measurement=current_state['zmp_y'],
            dt=dt
        )

        return {
            'pitch_torque': pitch_torque,
            'roll_torque': roll_torque,
            'zmp_x_adjustment': zmp_x_adjustment,
            'zmp_y_adjustment': zmp_y_adjustment
        }

# Import time at the top of the file
import time
```

### Part 3: Implementing ZMP-Based Balance Control (50 minutes)

Create ZMP-based balance control algorithms:

```python
#!/usr/bin/env python3
import numpy as np
import math

class ZMPController:
    def __init__(self, robot_height=0.8, gravity=9.81):
        self.robot_height = robot_height  # Height of center of mass
        self.gravity = gravity
        self.omega = math.sqrt(gravity / robot_height)  # Natural frequency

        # Support polygon parameters
        self.foot_width = 0.1  # meters
        self.foot_length = 0.15  # meters

        # ZMP tracking parameters
        self.zmp_tolerance = 0.05  # meters tolerance

    def calculate_zmp(self, com_x, com_y, com_z, com_vel_x, com_vel_y, com_acc_x, com_acc_y):
        """
        Calculate Zero Moment Point from Center of Mass state
        """
        zmp_x = com_x - (com_z / self.gravity) * com_acc_x
        zmp_y = com_y - (com_z / self.gravity) * com_acc_y

        return zmp_x, zmp_y

    def check_stability(self, zmp_x, zmp_y, support_center_x=0.0, support_center_y=0.0):
        """
        Check if ZMP is within support polygon
        """
        # Define support polygon bounds (simplified rectangular support)
        support_x_min = support_center_x - self.foot_width / 2
        support_x_max = support_center_x + self.foot_width / 2
        support_y_min = support_center_y - self.foot_length / 2
        support_y_max = support_center_y + self.foot_length / 2

        is_stable = (support_x_min <= zmp_x <= support_x_max and
                     support_y_min <= zmp_y <= support_y_max)

        return is_stable, {
            'x_bounds': (support_x_min, support_x_max),
            'y_bounds': (support_y_min, support_y_max)
        }

    def generate_com_trajectory(self, start_com, target_com, duration, dt=0.01):
        """
        Generate smooth CoM trajectory from start to target
        """
        steps = int(duration / dt)
        trajectory = []

        for i in range(steps):
            t = i * dt / duration  # Normalized time [0, 1]

            # Use quintic polynomial for smooth trajectory
            # This provides smooth position, velocity, and acceleration profiles
            scale = 6*t**5 - 15*t**4 + 10*t**3  # Smooth interpolation

            current_x = start_com[0] + scale * (target_com[0] - start_com[0])
            current_y = start_com[1] + scale * (target_com[1] - start_com[1])
            current_z = start_com[2] + scale * (target_com[2] - start_com[2])

            trajectory.append([current_x, current_y, current_z])

        return trajectory

    def calculate_capture_point(self, com_x, com_y, com_vel_x, com_vel_y):
        """
        Calculate capture point - where to step to stop completely
        """
        capture_x = com_x + com_vel_x / self.omega
        capture_y = com_y + com_vel_y / self.omega

        return capture_x, capture_y

    def balance_control_step(self, current_state, dt=0.01):
        """
        Perform one step of balance control
        current_state should contain:
        - com_position: [x, y, z] of center of mass
        - com_velocity: [vx, vy, vz] of center of mass
        - com_acceleration: [ax, ay, az] of center of mass
        - support_center: [x, y] of support polygon center
        """
        com_x, com_y, com_z = current_state['com_position']
        com_vx, com_vy, com_vz = current_state['com_velocity']
        com_ax, com_ay, com_az = current_state['com_acceleration']
        support_x, support_y = current_state['support_center']

        # Calculate current ZMP
        zmp_x, zmp_y = self.calculate_zmp(com_x, com_y, com_z, com_vx, com_vy, com_ax, com_ay)

        # Check stability
        is_stable, bounds = self.check_stability(zmp_x, zmp_y, support_x, support_y)

        # Calculate control outputs
        control_outputs = {}

        if not is_stable:
            # Generate corrective motion
            # Simple approach: move CoM towards stable region
            target_zmp_x = np.clip(zmp_x, bounds['x_bounds'][0] + 0.02, bounds['x_bounds'][1] - 0.02)
            target_zmp_y = np.clip(zmp_y, bounds['y_bounds'][0] + 0.02, bounds['y_bounds'][1] - 0.02)

            # Calculate target CoM position
            target_com_x = target_zmp_x + (com_z / self.gravity) * com_ax
            target_com_y = target_zmp_y + (com_z / self.gravity) * com_ay

            control_outputs = {
                'target_com_x': target_com_x,
                'target_com_y': target_com_y,
                'zmp_correction_x': target_zmp_x - zmp_x,
                'zmp_correction_y': target_zmp_y - zmp_y,
                'is_stable': False
            }
        else:
            control_outputs = {
                'target_com_x': com_x,
                'target_com_y': com_y,
                'zmp_correction_x': 0.0,
                'zmp_correction_y': 0.0,
                'is_stable': True
            }

        return control_outputs

class InvertedPendulumController:
    def __init__(self, height=0.8, gravity=9.81):
        self.height = height
        self.gravity = gravity
        self.natural_freq = math.sqrt(gravity / height)

        # PID gains for inverted pendulum control
        self.kp = 100.0  # Proportional gain
        self.kd = 20.0   # Derivative gain (damping)

    def control_torque(self, angle_error, angular_velocity):
        """
        Calculate control torque for inverted pendulum
        """
        torque = self.kp * angle_error + self.kd * angular_velocity
        return torque

    def linearize_dynamics(self, angle, angular_velocity):
        """
        Linearize the inverted pendulum dynamics around the upright position
        """
        # For small angles: sin(theta) ≈ theta
        # Dynamics: mgh*sin(theta) - u = mL^2*theta_ddot
        # Linearized: mgh*theta - u = mL^2*theta_ddot
        # Or: theta_ddot = (g/L)*theta - (1/mL^2)*u

        # Assuming unit mass and moment of inertia
        acceleration = (self.gravity / self.height) * angle - (1.0 / (self.height**2)) * self.control_torque(angle, angular_velocity)

        return acceleration
```

### Part 4: Creating the ROS2 Balance Control Node (40 minutes)

Create the main ROS2 node that implements balance control:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import Float32, Bool
from builtin_interfaces.msg import Time
import numpy as np
import math
from lab_balance_control.pid_controller import BalancePIDController
from lab_balance_control.zmp_controller import ZMPController

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Publishers
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.balance_status_pub = self.create_publisher(Bool, 'balance_status', 10)
        self.zmp_pub = self.create_publisher(Point, 'zmp_position', 10)

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # Initialize controllers
        self.balance_pid = BalancePIDController()
        self.zmp_controller = ZMPController(robot_height=0.8)

        # Robot state
        self.current_orientation = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w quaternion
        self.current_angular_velocity = [0.0, 0.0, 0.0]  # x, y, z
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_efforts = {}

        # Balance control parameters
        self.control_frequency = 100  # Hz
        self.dt = 1.0 / self.control_frequency
        self.balance_active = True
        self.fall_detected = False

        # Support polygon (simplified - assuming feet are centered under body)
        self.support_center = [0.0, 0.0]

        # Control timer
        self.control_timer = self.create_timer(1.0/self.control_frequency, self.balance_control_loop)

        self.get_logger().info('Balance controller initialized')

    def imu_callback(self, msg):
        """Process IMU data for balance control"""
        # Update orientation (quaternion)
        self.current_orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        # Update angular velocity
        self.current_angular_velocity = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]

    def joint_state_callback(self, msg):
        """Update joint state"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]
            if i < len(msg.effort):
                self.joint_efforts[name] = msg.effort[i]

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles"""
        x, y, z, w = q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def estimate_com_state(self):
        """Estimate Center of Mass state from available sensors"""
        # In a real implementation, this would use forward kinematics and IMU data
        # For this simulation, we'll make reasonable estimates

        roll, pitch, yaw = self.quaternion_to_euler(self.current_orientation)

        # Simplified CoM estimation (in reality, this requires full kinematic model)
        com_height = 0.75  # Approximate CoM height
        com_x = 0.0  # Initially centered
        com_y = 0.0  # Initially centered

        # Estimate CoM velocity from IMU
        com_vel_x = self.current_angular_velocity[1] * com_height  # From pitch rate
        com_vel_y = -self.current_angular_velocity[0] * com_height  # From roll rate
        com_vel_z = self.current_angular_velocity[2] * 0.01  # Small vertical motion

        # Estimate CoM acceleration
        # In reality, this would come from IMU readings
        com_acc_x = self.current_angular_velocity[1] * self.current_angular_velocity[2]  # Simplified
        com_acc_y = -self.current_angular_velocity[0] * self.current_angular_velocity[2]  # Simplified
        com_acc_z = 0.0  # Assume gravity dominates

        return {
            'position': [com_x, com_y, com_height],
            'velocity': [com_vel_x, com_vel_y, com_vel_z],
            'acceleration': [com_acc_x, com_acc_y, com_acc_z],
            'orientation': [roll, pitch, yaw],
            'angular_velocity': self.current_angular_velocity
        }

    def balance_control_loop(self):
        """Main balance control loop"""
        if not self.balance_active:
            return

        # Estimate current state
        current_state = self.estimate_com_state()

        # Calculate ZMP
        zmp_x, zmp_y = self.zmp_controller.calculate_zmp(
            current_state['position'][0], current_state['position'][1],
            current_state['position'][2], current_state['velocity'][0],
            current_state['velocity'][1], current_state['acceleration'][0],
            current_state['acceleration'][1]
        )

        # Check stability
        is_stable, bounds = self.zmp_controller.check_stability(
            zmp_x, zmp_y, self.support_center[0], self.support_center[1]
        )

        # Publish ZMP position
        zmp_msg = Point()
        zmp_msg.x = zmp_x
        zmp_msg.y = zmp_y
        zmp_msg.z = 0.0  # ZMP is on ground plane
        self.zmp_pub.publish(zmp_msg)

        # Publish balance status
        balance_status_msg = Bool()
        balance_status_msg.data = is_stable
        self.balance_status_pub.publish(balance_status_msg)

        # Calculate balance corrections
        balance_state = {
            'pitch_angle': current_state['orientation'][1],
            'roll_angle': current_state['orientation'][0],
            'pitch_rate': current_state['angular_velocity'][1],
            'roll_rate': current_state['angular_velocity'][0],
            'zmp_x': zmp_x,
            'zmp_y': zmp_y,
            'target_zmp_x': self.support_center[0],
            'target_zmp_y': self.support_center[1]
        }

        balance_outputs = self.balance_pid.calculate_balance_torques(balance_state)

        # Generate joint commands based on balance corrections
        joint_commands = self.calculate_joint_commands(balance_outputs, current_state)

        # Publish joint commands
        if joint_commands:
            self.joint_cmd_pub.publish(joint_commands)

        # Log stability status
        if not is_stable:
            self.get_logger().warn(f'Balance lost! ZMP: ({zmp_x:.3f}, {zmp_y:.3f}), Bounds: X({bounds["x_bounds"][0]:.3f}, {bounds["x_bounds"][1]:.3f}), Y({bounds["y_bounds"][0]:.3f}, {bounds["y_bounds"][1]:.3f})')
        else:
            self.get_logger().info(f'Balance maintained. ZMP: ({zmp_x:.3f}, {zmp_y:.3f})')

    def calculate_joint_commands(self, balance_outputs, current_state):
        """Calculate joint commands from balance outputs"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Define which joints are used for balance control
        balance_joints = [
            'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
            'torso_pitch', 'torso_roll'
        ]

        # Calculate joint positions based on balance corrections
        joint_positions = []

        for joint_name in balance_joints:
            # Simplified balance control mapping
            if 'hip_pitch' in joint_name:
                # Use pitch correction
                correction = balance_outputs['pitch_torque'] * 0.1
            elif 'ankle_pitch' in joint_name:
                # Use pitch correction with opposite sign for ankle
                correction = -balance_outputs['pitch_torque'] * 0.15
            elif 'hip_roll' in joint_name:
                # Use roll correction
                correction = balance_outputs['roll_torque'] * 0.1
            elif 'ankle_roll' in joint_name:
                # Use roll correction with opposite sign for ankle
                correction = -balance_outputs['roll_torque'] * 0.15
            elif 'torso' in joint_name:
                # Direct torso control for major corrections
                if 'pitch' in joint_name:
                    correction = balance_outputs['pitch_torque'] * 0.05
                else:  # roll
                    correction = balance_outputs['roll_torque'] * 0.05
            else:
                correction = 0.0  # Default no correction

            # Get current position and apply correction
            current_pos = self.joint_positions.get(joint_name, 0.0)
            new_pos = current_pos + correction

            # Apply reasonable limits
            if 'hip' in joint_name:
                new_pos = max(-0.5, min(0.5, new_pos))
            elif 'ankle' in joint_name:
                new_pos = max(-0.3, min(0.3, new_pos))
            elif 'torso' in joint_name:
                new_pos = max(-0.2, min(0.2, new_pos))

            joint_positions.append(new_pos)

        msg.name = balance_joints
        msg.position = joint_positions
        msg.velocity = [0.0] * len(joint_positions)  # Desired velocities
        msg.effort = [0.0] * len(joint_positions)   # Desired efforts

        return msg

    def detect_fall(self, orientation, angular_velocity):
        """Detect if robot has fallen based on orientation and angular velocity"""
        roll, pitch, _ = orientation

        # Check if angle exceeds safe limits (45 degrees)
        fall_threshold = math.radians(45)  # 45 degrees in radians
        angular_velocity_threshold = 2.0  # rad/s

        if (abs(roll) > fall_threshold or
            abs(pitch) > fall_threshold or
            abs(angular_velocity[0]) > angular_velocity_threshold or
            abs(angular_velocity[1]) > angular_velocity_threshold):
            return True

        return False

def main(args=None):
    rclpy.init(args=args)
    balance_controller = BalanceController()

    try:
        rclpy.spin(balance_controller)
    except KeyboardInterrupt:
        pass
    finally:
        balance_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Part 5: Implementing Stability Analysis Tools (30 minutes)

Create tools for analyzing system stability:

```python
#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import math

class StabilityAnalyzer:
    def __init__(self):
        self.time_data = []
        self.state_data = {
            'roll': [],
            'pitch': [],
            'zmp_x': [],
            'zmp_y': [],
            'com_x': [],
            'com_y': []
        }
        self.control_outputs = {
            'torque_roll': [],
            'torque_pitch': []
        }

    def add_data_point(self, time, state, control_out):
        """Add a data point to the analysis buffer"""
        self.time_data.append(time)

        # Add state data
        self.state_data['roll'].append(state.get('roll_angle', 0))
        self.state_data['pitch'].append(state.get('pitch_angle', 0))
        self.state_data['zmp_x'].append(state.get('zmp_x', 0))
        self.state_data['zmp_y'].append(state.get('zmp_y', 0))
        self.state_data['com_x'].append(state.get('com_x', 0))
        self.state_data['com_y'].append(state.get('com_y', 0))

        # Add control outputs
        self.control_outputs['torque_roll'].append(control_out.get('roll_torque', 0))
        self.control_outputs['torque_pitch'].append(control_out.get('pitch_torque', 0))

    def analyze_stability(self):
        """Analyze stability based on collected data"""
        if len(self.time_data) < 10:
            return "Insufficient data for analysis"

        # Calculate stability metrics
        roll_std = np.std(self.state_data['roll'])
        pitch_std = np.std(self.state_data['pitch'])

        # Check for increasing trends (instability)
        if len(self.time_data) > 20:
            recent_roll = self.state_data['roll'][-20:]
            recent_pitch = self.state_data['pitch'][-20:]

            # Calculate trends
            roll_trend = np.polyfit(range(len(recent_roll)), recent_roll, 1)[0]
            pitch_trend = np.polyfit(range(len(recent_pitch)), recent_pitch, 1)[0]
        else:
            roll_trend = 0
            pitch_trend = 0

        # Evaluate stability
        stability_metrics = {
            'roll_oscillation': roll_std,
            'pitch_oscillation': pitch_std,
            'roll_drift': abs(roll_trend),
            'pitch_drift': abs(pitch_trend),
            'max_roll_angle': max(abs(x) for x in self.state_data['roll']),
            'max_pitch_angle': max(abs(x) for x in self.state_data['pitch'])
        }

        # Overall stability assessment
        if (stability_metrics['roll_oscillation'] < 0.1 and
            stability_metrics['pitch_oscillation'] < 0.1 and
            stability_metrics['roll_drift'] < 0.01 and
            stability_metrics['pitch_drift'] < 0.01 and
            stability_metrics['max_roll_angle'] < 0.2 and
            stability_metrics['max_pitch_angle'] < 0.2):
            stability_assessment = "Stable"
        elif (stability_metrics['roll_oscillation'] < 0.3 and
              stability_metrics['pitch_oscillation'] < 0.3):
            stability_assessment = "Marginally Stable"
        else:
            stability_assessment = "Unstable"

        return {
            'assessment': stability_assessment,
            'metrics': stability_metrics
        }

    def plot_analysis(self):
        """Plot stability analysis"""
        if not self.time_data:
            print("No data to plot")
            return

        fig, axes = plt.subplots(3, 2, figsize=(15, 12))

        # Plot 1: Roll angle over time
        axes[0, 0].plot(self.time_data, self.state_data['roll'])
        axes[0, 0].set_title('Roll Angle Over Time')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Roll Angle (rad)')
        axes[0, 0].grid(True)

        # Plot 2: Pitch angle over time
        axes[0, 1].plot(self.time_data, self.state_data['pitch'])
        axes[0, 1].set_title('Pitch Angle Over Time')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Pitch Angle (rad)')
        axes[0, 1].grid(True)

        # Plot 3: ZMP trajectory
        axes[1, 0].plot(self.state_data['zmp_x'], self.state_data['zmp_y'], 'b-', label='ZMP Trajectory')
        axes[1, 0].scatter([0], [0], color='red', s=100, label='Origin', zorder=5)
        axes[1, 0].set_title('ZMP Trajectory')
        axes[1, 0].set_xlabel('ZMP X (m)')
        axes[1, 0].set_ylabel('ZMP Y (m)')
        axes[1, 0].grid(True)
        axes[1, 0].legend()
        axes[1, 0].axis('equal')

        # Plot 4: COM position
        axes[1, 1].plot(self.state_data['com_x'], self.state_data['com_y'], 'g-', label='CoM Path')
        axes[1, 1].set_title('Center of Mass Position')
        axes[1, 1].set_xlabel('COM X (m)')
        axes[1, 1].set_ylabel('COM Y (m)')
        axes[1, 1].grid(True)
        axes[1, 1].axis('equal')

        # Plot 5: Roll torque
        axes[2, 0].plot(self.time_data, self.control_outputs['torque_roll'])
        axes[2, 0].set_title('Roll Torque Applied')
        axes[2, 0].set_xlabel('Time (s)')
        axes[2, 0].set_ylabel('Torque (Nm)')
        axes[2, 0].grid(True)

        # Plot 6: Pitch torque
        axes[2, 1].plot(self.time_data, self.control_outputs['torque_pitch'])
        axes[2, 1].set_title('Pitch Torque Applied')
        axes[2, 1].set_xlabel('Time (s)')
        axes[2, 1].set_ylabel('Torque (Nm)')
        axes[2, 1].grid(True)

        plt.tight_layout()
        plt.show()

    def frequency_analysis(self):
        """Perform frequency domain analysis"""
        if len(self.state_data['roll']) < 100:
            return "Need more data for frequency analysis"

        # Calculate power spectral density for roll angle
        freqs, psd_roll = signal.welch(self.state_data['roll'], fs=100)  # Assuming 100Hz sampling

        # Find dominant frequencies
        peak_idx = np.argmax(psd_roll)
        dominant_freq = freqs[peak_idx]
        peak_power = psd_roll[peak_idx]

        return {
            'dominant_frequency': dominant_freq,
            'peak_power': peak_power,
            'frequency_spectrum': (freqs, psd_roll)
        }

class PIDTuner:
    def __init__(self, plant_model=None):
        """
        Simple PID tuner based on Ziegler-Nichols method
        For a first-order system with delay: G(s) = K * exp(-Ls) / (Ts + 1)
        """
        self.plant_model = plant_model

    def ziegler_nichols_step_response(self, step_response_data, time_vector):
        """
        Calculate PID parameters using Ziegler-Nichols method from step response
        """
        # Find the point of inflection (63.2% of final value)
        final_value = step_response_data[-1]
        target_28_3 = 0.283 * final_value
        target_63_2 = 0.632 * final_value

        # Find time constants
        t1 = None  # Time to reach 28.3%
        t2 = None  # Time to reach 63.2%

        for i, (t, val) in enumerate(zip(time_vector, step_response_data)):
            if t1 is None and val >= target_28_3:
                t1 = t
            if t2 is None and val >= target_63_2:
                t2 = t
            if t1 is not None and t2 is not None:
                break

        if t1 is not None and t2 is not None:
            L = 1.5 * t1 - 0.5 * t2  # Dead time estimate
            T = 2 * (t2 - t1)         # Time constant estimate
            K = final_value            # Gain

            # Ziegler-Nichols settings for PID
            kp = 1.2 * T / (K * L)
            ti = 2 * L
            td = 0.5 * L

            ki = kp / ti
            kd = kp * td

            return {
                'kp': kp,
                'ki': ki,
                'kd': kd,
                'model_params': {'K': K, 'T': T, 'L': L}
            }

        return None

    def critical_gain_method(self, plant_func, initial_kp=0.1, max_kp=100):
        """
        Find critical gain using iterative method
        This is a simplified version - in practice, you'd need to detect oscillations
        """
        # For this example, we'll return standard Ziegler-Nichols values
        # In a real implementation, you would run the system with increasing Kp
        # until sustained oscillations occur
        ku = 10.0  # Critical gain (example value)
        pu = 2.0   # Oscillation period (example value)

        # Ziegler-Nichols settings
        settings = {
            'P': {'kp': 0.5 * ku},
            'PI': {'kp': 0.45 * ku, 'ki': 0.45 * ku / (0.83 * pu)},
            'PID': {'kp': 0.6 * ku, 'ki': 1.2 * ku / pu, 'kd': 3 * ku * pu / 40}
        }

        return settings
```

### Part 6: Creating a Balance Testing Node (25 minutes)

Create a node to test balance control with various disturbances:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Imu
import time
import math

class BalanceTester(Node):
    def __init__(self):
        super().__init__('balance_tester')

        # Publishers for applying disturbances
        self.disturbance_pub = self.create_publisher(WrenchStamped, 'applied_force', 10)
        self.test_status_pub = self.create_publisher(String, 'test_status', 10)

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.balance_status_sub = self.create_subscription(Bool, 'balance_status', self.balance_status_callback, 10)

        # Test parameters
        self.test_sequence = [
            ('small_push', 5.0, 0.5),      # (test_name, force, duration)
            ('medium_push', 8.0, 0.8),
            ('large_push', 12.0, 1.0),
            ('pull_test', -6.0, 0.6),
            ('oscillatory', 4.0, 2.0),    # Applied oscillatory for 2 seconds
        ]

        self.current_test_index = 0
        self.test_active = False
        self.test_start_time = None
        self.test_duration = 0.0

        # Robot state
        self.balance_ok = True
        self.current_orientation = [0.0, 0.0, 0.0, 1.0]

        # Test timer
        self.test_timer = self.create_timer(0.1, self.run_test)

        self.get_logger().info('Balance tester initialized')

    def imu_callback(self, msg):
        """Update robot orientation from IMU"""
        self.current_orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

    def balance_status_callback(self, msg):
        """Update balance status"""
        self.balance_ok = msg.data

    def run_test(self):
        """Run balance tests"""
        current_time = time.time()

        if not self.test_active:
            if self.current_test_index < len(self.test_sequence):
                self.start_next_test()
            else:
                self.get_logger().info('All tests completed')
                return

        if self.test_active and self.test_start_time:
            elapsed_time = current_time - self.test_start_time

            if elapsed_time >= self.test_duration:
                self.end_current_test()
                return

            # Apply disturbance based on test type
            test_name = self.test_sequence[self.current_test_index][0]
            self.apply_disturbance(test_name, elapsed_time)

    def start_next_test(self):
        """Start the next test in sequence"""
        if self.current_test_index >= len(self.test_sequence):
            return

        test_name, force_mag, duration = self.test_sequence[self.current_test_index]

        self.test_active = True
        self.test_start_time = time.time()
        self.test_duration = duration

        status_msg = String()
        status_msg.data = f'Starting {test_name} test with {force_mag}N for {duration}s'
        self.test_status_pub.publish(status_msg)

        self.get_logger().info(f'Starting {test_name} test')

    def apply_disturbance(self, test_name, elapsed_time):
        """Apply the specified disturbance"""
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = 'base_link'

        if test_name == 'oscillatory':
            # Apply oscillatory force
            frequency = 2.0  # Hz
            force_x = 5.0 * math.sin(2 * math.pi * frequency * elapsed_time)
            force_y = 2.0 * math.cos(2 * math.pi * frequency * elapsed_time)
        else:
            # Apply constant force based on test name
            force_x = self.test_sequence[self.current_test_index][1]
            force_y = 0.0

        wrench_msg.wrench.force.x = force_x
        wrench_msg.wrench.force.y = force_y
        wrench_msg.wrench.force.z = 0.0
        wrench_msg.wrench.torque.x = 0.0
        wrench_msg.wrench.torque.y = 0.0
        wrench_msg.wrench.torque.z = 0.0

        self.disturbance_pub.publish(wrench_msg)

    def end_current_test(self):
        """End current test and move to next"""
        test_name = self.test_sequence[self.current_test_index][0]

        status_msg = String()
        status_msg.data = f'Completed {test_name} test - Balance: {"RECOVERED" if self.balance_ok else "LOST"}'
        self.test_status_pub.publish(status_msg)

        self.get_logger().info(f'Test {test_name} completed - Balance status: {"OK" if self.balance_ok else "LOST"}')

        # Reset for next test
        self.test_active = False
        self.test_start_time = None
        self.current_test_index += 1

        # Small delay before next test
        time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    tester = BalanceTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Part 7: Creating Launch File (10 minutes)

Create a launch file to run the complete balance control system:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab_balance_control',
            executable='balance_controller',
            name='balance_controller',
            parameters=[
                {'control_frequency': 100},
                {'robot_height': 0.8}
            ],
            output='screen'
        ),
        Node(
            package='lab_balance_control',
            executable='balance_tester',
            name='balance_tester',
            output='screen'
        )
    ])
```

### Part 8: Updating setup.py and Building (15 minutes)

Update the setup.py file:

```python
from setuptools import setup

package_name = 'lab_balance_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/balance_control_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Balance control for humanoid robots lab',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balance_controller = lab_balance_control.balance_controller:main',
            'balance_tester = lab_balance_control.balance_tester:main',
        ],
    },
)
```

Build the package:

```bash
cd ~/ros2_labs
colcon build --packages-select lab_balance_control
source install/setup.bash
```

## Discussion Questions

1. How do PID controllers handle the inherent instability of bipedal robots?

2. What are the advantages and disadvantages of ZMP-based balance control?

3. How does the choice of control frequency affect balance performance?

4. What safety considerations are important in balance control systems?

5. How would you adapt these control algorithms for different robot morphologies?

## Extension Activities

1. **Whole-Body Control**: Implement whole-body balance control using all available joints
2. **Walking Integration**: Extend balance control to include walking patterns
3. **Adaptive Control**: Implement self-tuning controllers that adapt to changing conditions
4. **Robust Control**: Design controllers that are robust to model uncertainties
5. **Machine Learning**: Use reinforcement learning to optimize balance controllers

## Assessment

Complete the following self-assessment:

- I can implement PID controllers for balance: [ ] Yes [ ] No [ ] Partially
- I understand ZMP theory and its application: [ ] Yes [ ] No [ ] Partially
- I can design feedback control systems: [ ] Yes [ ] No [ ] Partially
- I can analyze system stability: [ ] Yes [ ] No [ ] Partially
- I understand the challenges of humanoid balance: [ ] Yes [ ] No [ ] Partially

## Troubleshooting Tips

- **Oscillations**: Reduce PID gains, especially derivative gain
- **Slow Response**: Increase proportional gain
- **Steady-State Error**: Add integral action or increase integral gain
- **Actuator Saturation**: Implement anti-windup and gain scheduling
- **Sensor Noise**: Add filtering to control loop

## Conclusion

This lab has provided hands-on experience with balance control for humanoid robots, including PID control, ZMP-based methods, and stability analysis. You've learned to implement practical balance controllers and test their performance under various conditions.

## Resources for Further Exploration

- "Feedback Control of Dynamic Systems" by Franklin, Powell, and Emami-Naeini
- "Humanoid Robotics: A Reference" by Ambarish Goswami and Prahlad Vadakkepat
- "Robotics: Control, Sensing, Vision, and Intelligence" by Fu, Gonzalez, and Lee
- Research papers on balance control and humanoid robotics
- ROS2 Control documentation for advanced control techniques
- Simulation environments like Gazebo for testing control algorithms