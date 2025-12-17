---
title: Capstone Project - Building Your Own Robot
sidebar_position: 8
---

# Capstone Project: Building Your Own Robot

Welcome to the ultimate robotics challenge! This capstone project will integrate everything you've learned throughout this book. You'll design, simulate, and implement a complete robotic system that demonstrates your mastery of robotics concepts, from fundamental principles to advanced AI integration. This project will serve as a comprehensive portfolio piece showcasing your robotics skills.

## What You Will Learn

In this chapter, you'll:

- Apply the complete robot development lifecycle from concept to implementation
- Integrate mechanical design, electronics, control systems, and AI
- Work through real engineering challenges and trade-offs
- Create a fully functional robotic system with meaningful capabilities
- Document your development process and results
- Present and demonstrate your robot system
- Reflect on the learning journey and identify next steps

By the end of this project, you'll have a complete robotic system that demonstrates your understanding of the entire field of humanoid robotics.

## Project Overview

Your capstone project will involve designing and implementing a small humanoid robot (simulated or physical) capable of performing a specific task. The project is divided into phases that mirror real-world robotics development:

### Project Phases:
1. **Concept and Requirements**: Define what your robot will do
2. **Design and Simulation**: Create and test virtual robot
3. **Implementation**: Build and program the robot
4. **Testing and Validation**: Verify robot performance
5. **Documentation and Presentation**: Document and present your work

### Project Options:
- **Simulation-Only**: Create a sophisticated simulated robot using Gazebo
- **Hardware-Integrated**: Build/modify a physical robot (if hardware available)
- **Hybrid**: Simulated design with plans for physical implementation

## Phase 1: Concept and Requirements

### Define Your Robot's Purpose
Start by defining what your robot will do. Consider these project ideas:

#### Option 1: Service Robot
- **Function**: Navigate to deliver objects, avoid obstacles
- **Skills needed**: Path planning, navigation, manipulation
- **Success criteria**: Successfully deliver objects between locations

#### Option 2: Social Interaction Robot
- **Function**: Engage in basic conversations, recognize faces
- **Skills needed**: Computer vision, speech recognition, dialogue systems
- **Success criteria**: Successfully interact with humans for 5 minutes

#### Option 3: Object Sorting Robot
- **Function**: Identify and sort objects by color, shape, or size
- **Skills needed**: Computer vision, manipulation, decision making
- **Success criteria**: Correctly sort 10 objects with >90% accuracy

#### Option 4: Educational Robot
- **Function**: Demonstrate physics concepts through movement
- **Skills needed**: Motion control, kinematics, educational programming
- **Success criteria**: Demonstrate 3 physics concepts clearly

### Requirements Document Template:

```
Robot Name: [Your Robot Name]

Primary Function: [Main purpose]

Secondary Functions: [Additional capabilities]

Target Users: [Who will use this robot]

Technical Requirements:
- Locomotion: [Static, wheeled, bipedal, etc.]
- Sensors: [List required sensors]
- Manipulation: [Grippers, arms, etc.]
- Interaction: [Speech, gestures, etc.]
- Environment: [Indoor, outdoor, specific conditions]

Performance Requirements:
- Speed: [Movement speed]
- Accuracy: [Position accuracy, etc.]
- Reliability: [Mean time between failures]
- Runtime: [Battery life or operational time]

Success Criteria:
- Primary: [Main success metric]
- Secondary: [Additional metrics]
- Demo: [Live demonstration scenario]
```

## Phase 2: Design and Simulation

### Robot Design Process

#### 2.1 Mechanical Design
Design your robot's physical structure:

```xml
<!-- Example URDF for a simple robot -->
<?xml version="1.0"?>
<robot name="capstone_robot">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Add more links and joints as needed -->
  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint connecting head to base -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Add sensors -->
  <gazebo reference="base_link">
    <sensor type="camera" name="camera1">
      <visualize>true</visualize>
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>600</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>head</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

#### 2.2 Control Architecture Design
Plan your robot's control system:

```python
# Control architecture
class RobotController:
    """
    Main robot controller orchestrating all subsystems
    """
    def __init__(self):
        # Initialize subsystems
        self.navigation_controller = NavigationController()
        self.manipulation_controller = ManipulationController()
        self.vision_system = VisionSystem()
        self.dialogue_system = DialogueSystem()
        self.motion_controller = MotionController()

        # State machine
        self.state_machine = RobotStateMachine()

        # Communication interfaces
        self.ros_interface = ROSInterface()

    def execute_task(self, task_description):
        """Execute a high-level task"""
        # Parse task
        parsed_task = self.parse_task(task_description)

        # Plan sequence of actions
        action_sequence = self.plan_actions(parsed_task)

        # Execute each action
        for action in action_sequence:
            success = self.execute_action(action)
            if not success:
                return False

        return True
```

#### 2.3 Simulation Environment
Create a Gazebo world for your robot:

```xml
<!-- world file: capstone_world.world -->
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="capstone_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add objects for your robot to interact with -->
    <model name="table">
      <pose>2 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add objects for robot to manipulate -->
    <model name="object1">
      <pose>2.2 0.1 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Phase 3: Implementation

### 3.1 ROS2 Package Structure
Organize your project with a proper ROS2 package:

```
capstone_robot/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── controllers.yaml
│   └── robot.rviz
├── launch/
│   ├── robot.launch.py
│   └── simulation.launch.py
├── models/
│   └── capstone_robot/
│       ├── model.urdf
│       └── meshes/
├── worlds/
│   └── capstone_world.world
├── src/
│   ├── robot_controller.cpp
│   └── perception_node.cpp
├── scripts/
│   ├── setup_robot.sh
│   └── test_robot.py
└── test/
    └── test_navigation.py
```

### 3.2 Implementation Plan
Create an implementation timeline:

**Week 1**: Environment setup and basic robot model
**Week 2**: Basic movement and control
**Week 3**: Perception system integration
**Week 4**: Task execution and AI integration
**Week 5**: Testing and refinement
**Week 6**: Documentation and presentation

### 3.3 Core Implementation

#### Navigation Implementation:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Robot state
        self.current_pose = Pose()
        self.current_twist = Twist()
        self.scan_data = None

        # Navigation parameters
        self.target_pose = None
        self.navigation_active = False

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.navigation_control_loop)

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def scan_callback(self, msg):
        """Update laser scan data"""
        self.scan_data = msg

    def set_target(self, x, y):
        """Set navigation target"""
        self.target_pose = Pose()
        self.target_pose.position.x = x
        self.target_pose.position.y = y
        self.navigation_active = True

    def navigation_control_loop(self):
        """Main navigation control loop"""
        if not self.navigation_active or self.target_pose is None:
            return

        # Calculate distance to target
        dx = self.target_pose.position.x - self.current_pose.position.x
        dy = self.target_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        # Check if we've reached the target
        if distance < 0.1:  # 10cm threshold
            self.navigation_active = False
            self.stop_robot()
            self.get_logger().info("Reached target!")
            return

        # Calculate required rotation
        target_angle = math.atan2(dy, dx)
        current_angle = self.get_yaw_from_quaternion(self.current_pose.orientation)

        angle_diff = target_angle - current_angle
        # Normalize angle difference
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Create velocity command
        cmd = Twist()

        # Rotate towards target if significantly off
        if abs(angle_diff) > 0.2:  # 0.2 rad threshold
            cmd.angular.z = max(min(angle_diff * 1.0, 0.5), -0.5)  # Angular velocity
        else:
            # Move forward while maintaining direction
            cmd.linear.x = min(distance * 1.0, 0.5)  # Linear velocity
            cmd.angular.z = angle_diff * 2.0  # Small correction

        # Obstacle avoidance
        if self.scan_data:
            min_distance = min(self.scan_data.ranges)
            if min_distance < 0.5:  # 50cm safety distance
                cmd.linear.x = 0.0  # Stop if obstacle too close
                cmd.angular.z = 0.2  # Rotate away from obstacle

        self.cmd_vel_pub.publish(cmd)

    def get_yaw_from_quaternion(self, quat):
        """Extract yaw angle from quaternion"""
        import tf_transformations
        euler = tf_transformations.euler_from_quaternion([
            quat.x, quat.y, quat.z, quat.w
        ])
        return euler[2]  # yaw

    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = NavigationController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Vision System Implementation:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String

class VisionSystem(Node):
    def __init__(self):
        super().__init__('vision_system')

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        self.result_publisher = self.create_publisher(String, 'vision_result', 10)
        self.bridge = CvBridge()

        # Object detection parameters
        self.object_detection_enabled = True
        self.tracked_object = None

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # Process image based on active tasks
        if self.object_detection_enabled:
            processed_image, objects = self.detect_objects(cv_image)
        else:
            processed_image = cv_image
            objects = []

        # Publish results
        if objects:
            result_msg = String()
            result_msg.data = f"Detected {len(objects)} objects"
            self.result_publisher.publish(result_msg)

        # Optional: Publish processed image
        # processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
        # self.processed_image_pub.publish(processed_msg)

    def detect_objects(self, image):
        """Detect objects in image using color or feature detection"""
        # Convert BGR to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges (example: red objects)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter and process contours
        objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Filter small objects
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate center
                center_x = x + w // 2
                center_y = y + h // 2

                objects.append({
                    'type': 'red_object',
                    'center': (center_x, center_y),
                    'area': area,
                    'bbox': (x, y, w, h)
                })

                # Draw bounding box
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(image, f'Red: {area:.0f}', (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return image, objects

    def detect_faces(self, image):
        """Detect faces in image using Haar cascades"""
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(gray, 1.1, 4)

        for (x, y, w, h) in faces:
            cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)

        return len(faces) > 0  # Return True if faces detected

def main(args=None):
    rclpy.init(args=args)
    vision_system = VisionSystem()

    try:
        rclpy.spin(vision_system)
    except KeyboardInterrupt:
        pass
    finally:
        vision_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 4: Testing and Validation

### 4.1 Testing Framework
Create comprehensive tests for your robot:

```python
import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class TestRobotNavigation(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_robot_navigation')

        # Publishers and subscribers for testing
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.test_result_sub = self.node.create_subscription(
            String, 'test_result', self.test_result_callback, 10)

        self.test_result = None

    def test_navigation_to_target(self):
        """Test robot navigation to specific coordinates"""
        # Send navigation command
        nav_cmd = String()
        nav_cmd.data = "navigate_to:2.0,2.0"
        self.nav_pub.publish(nav_cmd)

        # Wait for completion
        timeout = time.time() + 60*2  # 2 minute timeout
        while self.test_result is None and time.time() < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.test_result)
        self.assertEqual(self.test_result.data, "navigation_success")

    def test_object_detection(self):
        """Test object detection capabilities"""
        # Place test objects in environment
        # Verify detection
        pass

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
```

### 4.2 Performance Validation
Test your robot against the original requirements:

- **Accuracy**: How well does it perform its tasks?
- **Reliability**: How often does it succeed vs. fail?
- **Speed**: How long does it take to complete tasks?
- **Robustness**: How well does it handle unexpected situations?

## Phase 5: Documentation and Presentation

### 5.1 Project Documentation
Create comprehensive documentation:

#### Technical Documentation:
- **System Architecture**: How all components work together
- **Installation Guide**: How to set up your robot system
- **User Manual**: How to operate the robot
- **Troubleshooting Guide**: Common issues and solutions
- **Code Documentation**: API documentation for your code

#### Development Process:
- **Design Decisions**: Why you chose specific approaches
- **Challenges Faced**: Problems encountered and solutions
- **Lessons Learned**: Key insights from the project
- **Future Improvements**: What could be enhanced

### 5.2 Presentation Materials
Prepare to demonstrate your robot:

#### Live Demonstration:
- Set up your robot and environment
- Show the robot performing its intended tasks
- Explain the technical implementation
- Handle questions from the audience

#### Presentation Slides:
- Project overview and objectives
- Technical approach and implementation
- Results and performance metrics
- Challenges and lessons learned
- Future work and improvements

## Mini Lab / Exercise

**Capstone Project Planning Workshop**

In this comprehensive exercise, you'll plan your own capstone project:

### Phase 1: Brainstorming (30 minutes)
1. List 5-10 robotics applications that interest you
2. For each, identify:
   - Primary function
   - Target users
   - Key technical challenges
   - Available resources needed

### Phase 2: Requirements Definition (45 minutes)
1. Choose your favorite application
2. Define specific, measurable requirements
3. Identify success criteria
4. Consider constraints and limitations

### Phase 3: Architecture Design (60 minutes)
1. Sketch the robot's physical design
2. Identify required sensors and actuators
3. Plan the software architecture
4. Consider the user interface

### Phase 4: Implementation Plan (30 minutes)
1. Break the project into 4-6 phases
2. Estimate time for each phase
3. Identify dependencies between phases
4. Plan testing and validation steps

### Questions for Reflection:
- How will you validate that your robot meets the requirements?
- What backup plans do you have if certain components don't work?
- How will you document your progress throughout the project?
- What metrics will you use to measure success?
- How will you handle integration challenges between subsystems?

## Next Chapter Preview

In the final chapter, "Next Steps in Robotics," we'll look ahead to your continued journey in robotics. You'll explore advanced topics for further study, learn about career opportunities in the field, discover research areas worth exploring, and understand how to stay current with rapidly evolving robotics technology.

We'll also provide guidance on building your robotics portfolio, contributing to open-source projects, and potentially starting your own robotics venture. This chapter will serve as your roadmap for continued growth and success in the exciting field of robotics.

You've completed an incredible journey learning about humanoid robotics, and now you're ready to take the next steps in your robotics career!

Get ready to continue your robotics journey!