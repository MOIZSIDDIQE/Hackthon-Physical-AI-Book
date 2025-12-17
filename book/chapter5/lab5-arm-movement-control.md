# Chapter 5 Lab: Arm Movement Control

## Objective

The goal of this lab is to implement and test arm movement control for a humanoid robot, focusing on inverse kinematics for reaching specific positions and trajectory planning for smooth motion execution. You will learn to control robotic arms to move to specified positions using both analytical and numerical inverse kinematics approaches.

## Learning Outcomes

By the end of this lab, you should be able to:

- Implement inverse kinematics algorithms for arm control
- Plan smooth trajectories for arm movements
- Control joint positions and velocities in a coordinated manner
- Use ROS2 to interface with robotic arm systems
- Validate arm movement accuracy and stability
- Handle constraints and singularities in arm kinematics

## Prerequisites

- ROS2 Humble Hawksbill installed
- Basic understanding of kinematics from Chapter 5
- Python programming skills
- Basic knowledge of linear algebra and trigonometry

## Materials Needed

- Computer with ROS2 installed
- Gazebo simulation environment
- Terminal access
- Text editor or IDE

## Background

Robotic arm control involves calculating the joint angles required to position the end-effector at desired locations. This lab will focus on implementing both analytical and numerical inverse kinematics solutions for a simplified 6-DOF robotic arm, which is typical for humanoid robot arms.

## Lab Procedure

### Part 1: Setting Up the Environment (15 minutes)

1. **Create a new ROS2 package for this lab:**
   ```bash
   mkdir -p ~/ros2_labs/src
   cd ~/ros2_labs/src
   ros2 pkg create --build-type ament_python lab_arm_control --dependencies rclpy geometry_msgs std_msgs sensor_msgs trajectory_msgs control_msgs
   ```

2. **Navigate to the package directory:**
   ```bash
   cd lab_arm_control
   ```

3. **Create the main script directory:**
   ```bash
   mkdir -p lab_arm_control
   ```

### Part 2: Understanding the Robot Arm Model (20 minutes)

For this lab, we'll work with a simplified 6-DOF arm model:

- **Joint 1**: Shoulder pan (rotation around z-axis)
- **Joint 2**: Shoulder lift (rotation around y-axis)
- **Joint 3**: Elbow (rotation around y-axis)
- **Joint 4**: Wrist 1 (rotation around y-axis)
- **Joint 5**: Wrist 2 (rotation around x-axis)
- **Joint 6**: Wrist 3 (rotation around y-axis)

The Denavit-Hartenberg parameters for this arm are:
- Link lengths: L1=0.2m, L2=0.3m, L3=0.3m (shoulder to elbow to wrist)
- Joint types: All revolute joints

### Part 3: Implementing Forward Kinematics (30 minutes)

Create the forward kinematics function to calculate end-effector position from joint angles:

```python
#!/usr/bin/env python3
import numpy as np
import math

def forward_kinematics(joint_angles):
    """
    Calculate end-effector position from joint angles using DH parameters
    joint_angles: list of 6 joint angles [theta1, theta2, theta3, theta4, theta5, theta6]
    Returns: [x, y, z, roll, pitch, yaw] of end-effector
    """
    # Simplified model for 6-DOF arm
    # Link lengths
    L1, L2, L3 = 0.2, 0.3, 0.3  # meters

    # Extract joint angles
    theta1, theta2, theta3, theta4, theta5, theta6 = joint_angles

    # Calculate end-effector position (simplified)
    # This is a simplified model - in real implementation, use full DH transformation
    x = (L2 * math.cos(theta2) + L3 * math.cos(theta2 + theta3)) * math.cos(theta1)
    y = (L2 * math.cos(theta2) + L3 * math.cos(theta2 + theta3)) * math.sin(theta1)
    z = L1 + L2 * math.sin(theta2) + L3 * math.sin(theta2 + theta3)

    return [x, y, z, theta4, theta5, theta6]

def rotation_matrix_to_rpy(R):
    """Convert rotation matrix to roll-pitch-yaw angles"""
    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])
```

### Part 4: Implementing Analytical Inverse Kinematics (45 minutes)

Create the analytical inverse kinematics solution for the 6-DOF arm:

```python
#!/usr/bin/env python3
import numpy as np
import math

def inverse_kinematics_analytical(target_pos, target_orientation=None):
    """
    Analytical inverse kinematics for 6-DOF arm
    target_pos: [x, y, z] target position
    target_orientation: [roll, pitch, yaw] target orientation (optional)
    Returns: list of joint angles or None if no solution found
    """
    x, y, z = target_pos

    # Simplified analytical solution for 6-DOF arm
    # Link lengths
    L1, L2, L3 = 0.2, 0.3, 0.3

    # Calculate theta1 (shoulder pan)
    theta1 = math.atan2(y, x)

    # Calculate distance from origin to projection of wrist center
    r = math.sqrt(x**2 + y**2)
    d = z - L1  # height above shoulder joint

    # Calculate distance from shoulder to wrist center
    D = math.sqrt(r**2 + d**2)

    # Check if target is reachable
    if D > L2 + L3:
        print("Target position is out of reach")
        return None

    if D < abs(L2 - L3):
        print("Target position is too close")
        return None

    # Calculate theta2 and theta3 (shoulder lift and elbow)
    cos_theta3 = (L2**2 + L3**2 - D**2) / (2 * L2 * L3)
    theta3 = math.acos(max(-1, min(1, cos_theta3)))  # Clamp to [-1, 1]

    # Calculate intermediate angle
    k1 = L2 + L3 * math.cos(theta3)
    k2 = L3 * math.sin(theta3)

    theta2 = math.atan2(d, r) - math.atan2(k2, k1)

    # For simplicity, set wrist angles to 0 (can be calculated if orientation is specified)
    theta4 = 0.0  # Wrist 1
    theta5 = 0.0  # Wrist 2
    theta6 = 0.0  # Wrist 3

    return [theta1, theta2, theta3, theta4, theta5, theta6]

def check_joint_limits(joint_angles, limits=None):
    """Check if joint angles are within limits"""
    if limits is None:
        # Default limits: [-pi, pi] for all joints
        limits = [(-math.pi, math.pi) for _ in range(6)]

    for i, (angle, (min_limit, max_limit)) in enumerate(zip(joint_angles, limits)):
        if angle < min_limit or angle > max_limit:
            return False
    return True
```

### Part 5: Implementing Numerical Inverse Kinematics (45 minutes)

Create a numerical inverse kinematics solution using the Jacobian method:

```python
#!/usr/bin/env python3
import numpy as np
import math

def jacobian_numerical(joint_angles, step=1e-6):
    """
    Calculate Jacobian matrix numerically
    joint_angles: current joint angles
    step: small step for numerical differentiation
    Returns: 6x6 Jacobian matrix
    """
    n = len(joint_angles)
    J = np.zeros((6, n))

    # Calculate forward kinematics at current position
    current_pos = forward_kinematics(joint_angles)

    for i in range(n):
        # Perturb joint i
        angles_plus = joint_angles.copy()
        angles_plus[i] += step

        # Calculate forward kinematics with perturbed joint
        pos_plus = forward_kinematics(angles_plus)

        # Calculate difference (numerical derivative)
        diff = np.array(pos_plus) - np.array(current_pos)
        J[:, i] = diff / step

    return J

def inverse_kinematics_numerical(target_pos, current_angles, max_iterations=100, tolerance=1e-4):
    """
    Numerical inverse kinematics using Jacobian transpose method
    target_pos: [x, y, z, roll, pitch, yaw] target pose
    current_angles: starting joint angles
    max_iterations: maximum number of iterations
    tolerance: position tolerance
    Returns: joint angles that achieve target position
    """
    joint_angles = np.array(current_angles, dtype=float)

    for iteration in range(max_iterations):
        # Calculate current position
        current_pos = forward_kinematics(joint_angles)

        # Calculate error
        error = np.array(target_pos) - np.array(current_pos)

        # Check if error is within tolerance
        if np.linalg.norm(error[:3]) < tolerance:  # Only check position, not orientation
            print(f"Converged after {iteration} iterations")
            return joint_angles.tolist()

        # Calculate Jacobian
        J = jacobian_numerical(joint_angles)

        # Use pseudo-inverse for better numerical stability
        J_pinv = np.linalg.pinv(J)

        # Update joint angles
        delta_angles = J_pinv @ error
        joint_angles += 0.1 * delta_angles  # Small step size for stability

    print(f"Did not converge after {max_iterations} iterations")
    return joint_angles.tolist()

def forward_kinematics(joint_angles):
    """
    Simplified forward kinematics for demonstration
    This is a placeholder - implement full DH transformation in practice
    """
    # This is a simplified version - in real implementation, use DH parameters
    L1, L2, L3 = 0.2, 0.3, 0.3

    theta1, theta2, theta3, theta4, theta5, theta6 = joint_angles

    x = (L2 * math.cos(theta2) + L3 * math.cos(theta2 + theta3)) * math.cos(theta1)
    y = (L2 * math.cos(theta2) + L3 * math.cos(theta2 + theta3)) * math.sin(theta1)
    z = L1 + L2 * math.sin(theta2) + L3 * math.sin(theta2 + theta3)

    return [x, y, z, theta4, theta5, theta6]
```

### Part 6: Creating the ROS2 Arm Controller Node (30 minutes)

Create the main ROS2 node that will control the arm:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point
from std_msgs.msg import String
import numpy as np
import math
from scipy.interpolate import interp1d

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 'arm_controller/joint_trajectory', 10)

        # Subscribers
        self.target_sub = self.create_subscription(
            Point, 'arm_target', self.target_callback, 10)

        # Parameters
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Current joint positions
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.target_position = None
        self.trajectory_active = False
        self.trajectory_index = 0

    def target_callback(self, msg):
        """Receive target position and plan trajectory"""
        target_pos = [msg.x, msg.y, msg.z]

        # Calculate inverse kinematics
        joint_angles = self.calculate_inverse_kinematics(target_pos)

        if joint_angles is not None:
            # Generate smooth trajectory
            self.generate_trajectory(joint_angles)
            self.get_logger().info(f'Moving arm to: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}]')
        else:
            self.get_logger().warn('Target position is not reachable')

    def calculate_inverse_kinematics(self, target_pos):
        """Calculate inverse kinematics for target position"""
        # Use analytical method first
        joint_angles = inverse_kinematics_analytical(target_pos)

        if joint_angles is None:
            # Fallback to numerical method
            current_angles = self.current_angles
            target_with_orientation = target_pos + [0, 0, 0]  # Add placeholder for orientation
            joint_angles = inverse_kinematics_numerical(
                target_with_orientation, current_angles)

        return joint_angles

    def generate_trajectory(self, target_angles):
        """Generate smooth trajectory from current to target angles"""
        # Current angles
        current_angles = np.array(self.current_angles)
        target_angles = np.array(target_angles)

        # Time vector (2 seconds for movement)
        duration = 2.0
        dt = 0.05  # 20 Hz
        time_steps = int(duration / dt)
        time_vector = np.linspace(0, duration, time_steps)

        # Generate smooth trajectory using cubic interpolation
        trajectory_points = []

        for i in range(6):  # For each joint
            # Cubic polynomial for smooth motion
            start_angle = current_angles[i]
            end_angle = target_angles[i]

            # Cubic interpolation: a + bt + ct^2 + dt^3
            # With boundary conditions: at t=0, position=start, velocity=0
            # At t=duration, position=end, velocity=0
            a = start_angle
            b = 0  # Initial velocity = 0
            c = 3 * (end_angle - start_angle) / (duration**2)
            d = -2 * (end_angle - start_angle) / (duration**3)

            joint_trajectory = a + b*time_vector + c*(time_vector**2) + d*(time_vector**3)

            if i == 0:
                trajectory_points = [list(point) for point in np.column_stack([joint_trajectory] * 6)]
            else:
                for j, point in enumerate(trajectory_points):
                    point[i] = joint_trajectory[j]

        # Create ROS2 trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()

        for t_idx, t in enumerate(time_vector):
            point = JointTrajectoryPoint()
            point.positions = trajectory_points[t_idx]
            point.velocities = [0.0] * 6  # Will calculate in real implementation
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            trajectory_msg.points.append(point)

        # Publish trajectory
        self.trajectory_pub.publish(trajectory_msg)
        self.trajectory_active = True
        self.target_trajectory = trajectory_msg

    def control_loop(self):
        """Main control loop"""
        if self.trajectory_active and self.trajectory_index < len(self.target_trajectory.points):
            # Publish current trajectory point
            point = self.target_trajectory.points[self.trajectory_index]
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = self.joint_names
            trajectory_msg.header.stamp = self.get_clock().now().to_msg()
            trajectory_msg.points = [point]

            self.trajectory_pub.publish(trajectory_msg)

            # Update current angles
            self.current_angles = list(point.positions)
            self.trajectory_index += 1

            if self.trajectory_index >= len(self.target_trajectory.points):
                self.trajectory_active = False
                self.trajectory_index = 0
                self.get_logger().info('Trajectory completed')

def inverse_kinematics_analytical(target_pos):
    """Analytical inverse kinematics implementation"""
    x, y, z = target_pos

    # Simplified analytical solution for 6-DOF arm
    L1, L2, L3 = 0.2, 0.3, 0.3

    # Calculate theta1 (shoulder pan)
    theta1 = math.atan2(y, x)

    # Calculate distance from origin to projection of wrist center
    r = math.sqrt(x**2 + y**2)
    d = z - L1  # height above shoulder joint

    # Calculate distance from shoulder to wrist center
    D = math.sqrt(r**2 + d**2)

    # Check if target is reachable
    if D > L2 + L3:
        print("Target position is out of reach")
        return None

    if D < abs(L2 - L3):
        print("Target position is too close")
        return None

    # Calculate theta2 and theta3 (shoulder lift and elbow)
    cos_theta3 = (L2**2 + L3**2 - D**2) / (2 * L2 * L3)
    theta3 = math.acos(max(-1, min(1, cos_theta3)))  # Clamp to [-1, 1]

    # Calculate intermediate angle
    k1 = L2 + L3 * math.cos(theta3)
    k2 = L3 * math.sin(theta3)

    theta2 = math.atan2(d, r) - math.atan2(k2, k1)

    # For simplicity, set wrist angles to 0
    theta4 = 0.0  # Wrist 1
    theta5 = 0.0  # Wrist 2
    theta6 = 0.0  # Wrist 3

    return [theta1, theta2, theta3, theta4, theta5, theta6]

def inverse_kinematics_numerical(target_pos, current_angles, max_iterations=100, tolerance=1e-4):
    """Numerical inverse kinematics implementation"""
    # This is a simplified version - full implementation would include Jacobian calculation
    # For this lab, we'll return the analytical solution as fallback
    return inverse_kinematics_analytical(target_pos[:3])

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()

    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Part 7: Creating a Test Node (20 minutes)

Create a test node to send target positions to the arm controller:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time

class ArmTestNode(Node):
    def __init__(self):
        super().__init__('arm_test_node')

        self.target_pub = self.create_publisher(Point, 'arm_target', 10)

        # Test positions
        self.test_positions = [
            [0.5, 0.0, 0.5],   # Position 1: Reach forward
            [0.3, 0.3, 0.4],   # Position 2: Reach right and forward
            [0.3, -0.3, 0.4],  # Position 3: Reach left and forward
            [0.4, 0.0, 0.3],   # Position 4: Lower position
            [0.0, 0.0, 0.5],   # Position 5: Return to center
        ]

        self.current_position_idx = 0

        # Timer to send test positions
        self.timer = self.create_timer(5.0, self.send_next_position)

    def send_next_position(self):
        """Send next test position"""
        if self.current_position_idx < len(self.test_positions):
            pos = self.test_positions[self.current_position_idx]
            target_msg = Point()
            target_msg.x = pos[0]
            target_msg.y = pos[1]
            target_msg.z = pos[2]

            self.target_pub.publish(target_msg)
            self.get_logger().info(f'Sent target position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]')

            self.current_position_idx += 1
        else:
            self.get_logger().info('Completed all test positions')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    test_node = ArmTestNode()

    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Part 8: Creating Launch File (15 minutes)

Create a launch file to run the complete system:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab_arm_control',
            executable='arm_controller',
            name='arm_controller',
            output='screen'
        ),
        Node(
            package='lab_arm_control',
            executable='arm_test_node',
            name='arm_test_node',
            output='screen'
        )
    ])
```

### Part 9: Updating setup.py (10 minutes)

Update the setup.py file to make the Python scripts executable:

```python
from setuptools import setup

package_name = 'lab_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arm_control_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Arm movement control lab',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = lab_arm_control.arm_controller:main',
            'arm_test_node = lab_arm_control.arm_test_node:main',
        ],
    },
)
```

### Part 10: Building and Testing (25 minutes)

1. **Build the package:**
   ```bash
   cd ~/ros2_labs
   colcon build --packages-select lab_arm_control
   source install/setup.bash
   ```

2. **Run the system:**
   ```bash
   ros2 launch lab_arm_control arm_control_launch.py
   ```

3. **Test with custom positions:**
   ```bash
   # In another terminal, send custom target positions
   ros2 topic pub /arm_target geometry_msgs/Point "{x: 0.4, y: 0.2, z: 0.5}"
   ```

## Discussion Questions

1. How do analytical and numerical inverse kinematics methods compare in terms of accuracy and computational requirements?

2. What are the main challenges in implementing smooth trajectory planning for robotic arms?

3. How do joint limits affect the inverse kinematics solution?

4. What safety considerations are important when controlling robotic arms?

5. How would you extend this system to handle orientation (roll, pitch, yaw) in addition to position?

## Extension Activities

1. **Advanced Trajectory Planning**: Implement minimum jerk trajectories for smoother motion
2. **Obstacle Avoidance**: Add collision detection and avoidance to the trajectory planning
3. **Visual Servoing**: Use camera feedback to improve positioning accuracy
4. **Force Control**: Implement compliant control for safe interaction
5. **Learning-based Control**: Use machine learning to improve inverse kinematics

## Assessment

Complete the following self-assessment:

- I can implement analytical inverse kinematics for a 6-DOF arm: [ ] Yes [ ] No [ ] Partially
- I understand numerical inverse kinematics methods: [ ] Yes [ ] No [ ] Partially
- I can plan smooth trajectories for arm movements: [ ] Yes [ ] No [ ] Partially
- I can integrate the system with ROS2: [ ] Yes [ ] No [ ] Partially
- I understand the challenges of arm control: [ ] Yes [ ] No [ ] Partially

## Troubleshooting Tips

- **Singularity Issues**: Check for gimbal lock and implement singularity handling
- **Joint Limit Violations**: Always check joint limits before sending commands
- **Convergence Problems**: Adjust step sizes and iteration limits in numerical methods
- **Timing Issues**: Ensure control loop runs at appropriate frequency
- **Coordinate Frame Issues**: Verify all coordinate systems are properly defined

## Conclusion

This lab has provided hands-on experience with robotic arm control, including inverse kinematics, trajectory planning, and ROS2 integration. You've learned to implement both analytical and numerical approaches to arm control and understand the trade-offs between different methods.

## Resources for Further Exploration

- "Robotics: Modelling, Planning and Control" by Siciliano and Khatib
- ROS2 Control documentation
- MoveIt! motion planning framework
- Research papers on inverse kinematics and trajectory planning
- OpenRAVE and PyKDL for advanced kinematics