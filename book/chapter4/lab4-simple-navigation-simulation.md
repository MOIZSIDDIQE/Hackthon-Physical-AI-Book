# Chapter 4 Lab: Simple Navigation Simulation

## Objective

The goal of this lab is to create a simple robot navigation simulation in Gazebo where a robot navigates through an obstacle course. You will learn to create custom worlds, spawn robots, implement basic navigation algorithms, and visualize robot behavior in simulation.

## Learning Outcomes

By the end of this lab, you should be able to:

- Create custom Gazebo world files with obstacles
- Spawn and control robots in Gazebo simulation
- Implement basic navigation algorithms (go-to-goal, obstacle avoidance)
- Use ROS2 navigation stack in simulation
- Visualize robot paths and sensor data
- Debug and tune navigation parameters

## Prerequisites

- ROS2 Humble Hawksbill installed
- Gazebo (or Gazebo Garden) installed
- Basic understanding of ROS2 concepts
- Basic Python programming skills

## Materials Needed

- Computer with ROS2 and Gazebo installed
- Terminal access
- Text editor or IDE
- Basic understanding of URDF and SDF formats

## Background

Navigation in robotics involves planning paths and controlling robot motion to reach desired destinations while avoiding obstacles. Simulation provides a safe environment to test navigation algorithms before deploying them on real robots. This lab will guide you through creating a simple navigation scenario with both reactive (obstacle avoidance) and deliberative (path planning) navigation approaches.

## Lab Procedure

### Part 1: Setting Up the Workspace (15 minutes)

1. **Create a new ROS2 workspace for simulation:**
   ```bash
   mkdir -p ~/ros2_sim_ws/src
   cd ~/ros2_sim_ws
   ```

2. **Create a package for this lab:**
   ```bash
   cd src
   ros2 pkg create --build-type ament_python lab_navigation_simulation --dependencies rclpy geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros tf2_geometry_msgs
   ```

3. **Navigate to the package directory:**
   ```bash
   cd lab_navigation_simulation
   ```

### Part 2: Creating a Simple Robot Model (25 minutes)

1. **Create the robot description directory:**
   ```bash
   mkdir -p urdf
   ```

2. **Create a simple differential drive robot URDF (`urdf/simple_robot.urdf`):**
   ```xml
   <?xml version="1.0"?>
   <robot name="simple_robot">
     <!-- Base Link -->
     <link name="base_link">
       <visual>
         <geometry>
           <cylinder length="0.2" radius="0.3"/>
         </geometry>
         <material name="blue">
           <color rgba="0 0 1 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.2" radius="0.3"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="5.0"/>
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
       </inertial>
     </link>

     <!-- Left Wheel -->
     <link name="left_wheel">
       <visual>
         <geometry>
           <cylinder length="0.05" radius="0.1"/>
         </geometry>
         <material name="black">
           <color rgba="0 0 0 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.05" radius="0.1"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
       </inertial>
     </link>

     <!-- Right Wheel -->
     <link name="right_wheel">
       <visual>
         <geometry>
           <cylinder length="0.05" radius="0.1"/>
         </geometry>
         <material name="black">
           <color rgba="0 0 0 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.05" radius="0.1"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <inertial>
           <mass value="0.5"/>
           <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
         </inertial>
       </inertial>
     </link>

     <!-- Joints -->
     <joint name="left_wheel_joint" type="continuous">
       <parent link="base_link"/>
       <child link="left_wheel"/>
       <origin xyz="0 0.25 -0.1" rpy="1.5708 0 0"/>
       <axis xyz="0 0 1"/>
     </joint>

     <joint name="right_wheel_joint" type="continuous">
       <parent link="base_link"/>
       <child link="right_wheel"/>
       <origin xyz="0 -0.25 -0.1" rpy="1.5708 0 0"/>
       <axis xyz="0 0 1"/>
     </joint>

     <!-- Differential Drive Plugin Configuration -->
     <gazebo>
       <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
         <left_joint>left_wheel_joint</left_joint>
         <right_joint>right_wheel_joint</right_joint>
         <wheel_separation>0.5</wheel_separation>
         <wheel_diameter>0.2</wheel_diameter>
         <max_wheel_torque>20</max_wheel_torque>
         <max_wheel_acceleration>1.0</max_wheel_acceleration>
         <command_topic>cmd_vel</command_topic>
         <odometry_topic>odom</odometry_topic>
         <odometry_frame>odom</odometry_frame>
         <robot_base_frame>base_link</robot_base_frame>
         <publish_odom>true</publish_odom>
         <publish_wheel_tf>false</publish_wheel_tf>
         <publish_wheel_joint_state>true</publish_wheel_joint_state>
         <ros>
           <namespace>/</namespace>
         </ros>
       </plugin>
     </gazebo>
   </robot>
   ```

### Part 3: Creating a Custom World (20 minutes)

1. **Create the worlds directory:**
   ```bash
   mkdir -p worlds
   ```

2. **Create a simple world with obstacles (`worlds/simple_obstacle_course.world`):**
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="simple_obstacle_course">
       <!-- Include sun and ground plane -->
       <include>
         <uri>model://sun</uri>
       </include>

       <include>
         <uri>model://ground_plane</uri>
       </include>

       <!-- Wall obstacles -->
       <model name="wall_1">
         <pose>3 0 0 0 0 0</pose>
         <link name="wall_1_link">
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.1 6 1</size>
               </box>
             </geometry>
             <material>
               <ambient>0.5 0.5 0.5 1</ambient>
               <diffuse>0.5 0.5 0.5 1</diffuse>
             </material>
           </visual>
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.1 6 1</size>
               </box>
             </geometry>
           </collision>
           <inertial>
             <mass>100</mass>
             <inertia>
               <ixx>1</ixx>
               <iyy>1</iyy>
               <izz>1</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <model name="wall_2">
         <pose>-3 0 0 0 0 0</pose>
         <link name="wall_2_link">
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.1 6 1</size>
               </box>
             </geometry>
             <material>
               <ambient>0.5 0.5 0.5 1</ambient>
               <diffuse>0.5 0.5 0.5 1</diffuse>
             </material>
           </visual>
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.1 6 1</size>
               </box>
             </geometry>
           </collision>
           <inertial>
             <mass>100</mass>
             <inertia>
               <ixx>1</ixx>
               <iyy>1</iyy>
               <izz>1</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <!-- Center obstacle -->
       <model name="center_obstacle">
         <pose>0 2 0 0 0 0</pose>
         <link name="center_obstacle_link">
           <visual name="visual">
             <geometry>
               <box>
                 <size>2 0.1 0.5</size>
               </box>
             </geometry>
             <material>
               <ambient>1 0 0 1</ambient>
               <diffuse>1 0 0 1</diffuse>
             </material>
           </visual>
           <collision name="collision">
             <geometry>
               <box>
                 <size>2 0.1 0.5</size>
               </box>
             </geometry>
           </collision>
           <inertial>
             <mass>50</mass>
             <inertia>
               <ixx>1</ixx>
               <iyy>1</iyy>
               <izz>1</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <!-- Goal marker -->
       <model name="goal_marker">
         <pose>4 4 0.25 0 0 0</pose>
         <link name="goal_marker_link">
           <visual name="visual">
             <geometry>
               <cylinder>
                 <radius>0.2</radius>
                 <length>0.5</length>
               </cylinder>
             </geometry>
             <material>
               <ambient>0 1 0 0.5</ambient>
               <diffuse>0 1 0 0.5</diffuse>
             </material>
           </visual>
           <collision name="collision">
             <geometry>
               <cylinder>
                 <radius>0.2</radius>
                 <length>0.5</length>
               </cylinder>
             </geometry>
           </collision>
           <inertial>
             <mass>1</mass>
             <inertia>
               <ixx>0.1</ixx>
               <iyy>0.1</iyy>
               <izz>0.1</izz>
             </inertia>
           </inertial>
         </link>
       </model>
     </world>
   </sdf>
   ```

### Part 4: Creating a Navigation Node (30 minutes)

1. **Create the navigation script directory:**
   ```bash
   mkdir -p lab_navigation_simulation
   ```

2. **Create the navigation node (`lab_navigation_simulation/navigation_node.py`):**
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Twist, Point
   from nav_msgs.msg import Odometry
   from sensor_msgs.msg import LaserScan
   from tf2_ros import TransformException
   from tf2_ros.buffer import Buffer
   from tf2_ros.transform_listener import TransformListener
   import math
   import numpy as np


   class SimpleNavigation(Node):
       def __init__(self):
           super().__init__('simple_navigation')

           # Publisher for velocity commands
           self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

           # Subscriber for odometry
           self.odom_sub = self.create_subscription(
               Odometry, 'odom', self.odom_callback, 10)

           # Subscriber for laser scan (for obstacle detection)
           self.scan_sub = self.create_subscription(
               LaserScan, 'scan', self.scan_callback, 10)

           # Timer for control loop
           self.timer = self.create_timer(0.1, self.control_loop)

           # Robot state
           self.current_pose = Point()
           self.current_yaw = 0.0
           self.obstacle_detected = False
           self.obstacle_distance = float('inf')

           # Navigation state
           self.goal_x = 4.0  # Goal position
           self.goal_y = 4.0
           self.reached_goal = False

           # Control parameters
           self.linear_speed = 0.5
           self.angular_speed = 0.5
           self.goal_tolerance = 0.3
           self.obstacle_threshold = 0.8

           self.get_logger().info('Simple Navigation Node Started')

       def odom_callback(self, msg):
           """Update robot position from odometry"""
           self.current_pose.x = msg.pose.pose.position.x
           self.current_pose.y = msg.pose.pose.position.y

           # Extract yaw from quaternion
           orientation = msg.pose.pose.orientation
           self.current_yaw = math.atan2(
               2 * (orientation.w * orientation.z + orientation.x * orientation.y),
               1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
           )

       def scan_callback(self, msg):
           """Process laser scan data for obstacle detection"""
           # Get distances in front of robot (narrow beam)
           front_scan = msg.ranges[len(msg.ranges)//2 - 10 : len(msg.ranges)//2 + 10]
           front_scan = [d for d in front_scan if not math.isnan(d) and d > 0]

           if front_scan:
               self.obstacle_distance = min(front_scan)
               self.obstacle_detected = self.obstacle_distance < self.obstacle_threshold
           else:
               self.obstacle_detected = False
               self.obstacle_distance = float('inf')

       def calculate_angle_to_goal(self):
           """Calculate the angle to the goal from current position"""
           dx = self.goal_x - self.current_pose.x
           dy = self.goal_y - self.current_pose.y
           goal_angle = math.atan2(dy, dx)
           return goal_angle

       def control_loop(self):
           """Main navigation control loop"""
           if self.reached_goal:
               # Stop robot when goal is reached
               cmd = Twist()
               self.cmd_vel_pub.publish(cmd)
               return

           # Calculate angle to goal
           angle_to_goal = self.calculate_angle_to_goal()
           angle_diff = angle_to_goal - self.current_yaw

           # Normalize angle to [-pi, pi]
           while angle_diff > math.pi:
               angle_diff -= 2 * math.pi
           while angle_diff < -math.pi:
               angle_diff += 2 * math.pi

           cmd = Twist()

           if self.obstacle_detected:
               # Obstacle avoidance behavior
               self.get_logger().info('Obstacle detected! Avoiding...')
               cmd.linear.x = 0.0
               cmd.angular.z = self.angular_speed  # Turn away from obstacle
           else:
               # Go-to-goal behavior
               if abs(angle_diff) > 0.1:  # If not facing goal
                   cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 2 * angle_diff))
                   cmd.linear.x = 0.0
               else:
                   # Move toward goal
                   cmd.linear.x = self.linear_speed
                   cmd.angular.z = 0.0

           # Check if goal is reached
           distance_to_goal = math.sqrt(
               (self.goal_x - self.current_pose.x)**2 +
               (self.goal_y - self.current_pose.y)**2
           )

           if distance_to_goal < self.goal_tolerance:
               self.reached_goal = True
               cmd.linear.x = 0.0
               cmd.angular.z = 0.0
               self.get_logger().info('Goal reached!')

           self.cmd_vel_pub.publish(cmd)


   def main(args=None):
       rclpy.init(args=args)
       navigation_node = SimpleNavigation()

       try:
           rclpy.spin(navigation_node)
       except KeyboardInterrupt:
           pass
       finally:
           # Stop the robot on shutdown
           cmd = Twist()
           navigation_node.cmd_vel_pub.publish(cmd)
           navigation_node.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Part 5: Creating a Launch File (15 minutes)

1. **Create the launch directory:**
   ```bash
   mkdir -p launch
   ```

2. **Create the launch file (`launch/navigation_simulation_launch.py`):**
   ```python
   import os
   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription, ExecuteProcess
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory


   def generate_launch_description():
       # Package directories
       pkg_name = 'lab_navigation_simulation'
       pkg_share = get_package_share_directory(pkg_name)

       # World file path
       world_file = os.path.join(pkg_share, 'worlds', 'simple_obstacle_course.world')

       # Robot URDF file path
       robot_urdf = os.path.join(pkg_share, 'urdf', 'simple_robot.urdf')

       # Launch Gazebo
       gazebo = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               get_package_share_directory('gazebo_ros'),
               '/launch/gzserver.launch.py'
           ]),
           launch_arguments={
               'world': world_file,
               'verbose': 'true'
           }.items()
       )

       gazebo_client = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               get_package_share_directory('gazebo_ros'),
               '/launch/gzclient.launch.py'
           ])
       )

       # Spawn robot in Gazebo
       spawn_robot = Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=[
               '-entity', 'simple_robot',
               '-file', robot_urdf,
               '-x', '0', '-y', '0', '-z', '0.2'
           ],
           output='screen'
       )

       # Robot state publisher
       robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           output='screen',
           parameters=[{
               'use_sim_time': True,
               'robot_description': open(robot_urdf).read()
           }]
       )

       # Navigation node
       navigation_node = Node(
           package='lab_navigation_simulation',
           executable='navigation_node',
           name='navigation_node',
           output='screen'
       )

       # RViz for visualization (optional)
       rviz = Node(
           package='rviz2',
           executable='rviz2',
           arguments=['-d', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'rviz', 'turtlebot3_gazebo.rviz')]
       )

       return LaunchDescription([
           gazebo,
           gazebo_client,
           spawn_robot,
           robot_state_publisher,
           navigation_node
           # Uncomment the next line if you have RViz configured
           # rviz
       ])
   ```

### Part 6: Building and Running the Simulation (25 minutes)

1. **Update the setup.py file to make the Python script executable:**
   ```python
   from setuptools import setup

   package_name = 'lab_navigation_simulation'

   setup(
       name=package_name,
       version='0.0.0',
       packages=[package_name],
       data_files=[
           ('share/ament_index/resource_index/packages',
           ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           ('share/' + package_name + '/urdf', ['urdf/simple_robot.urdf']),
           ('share/' + package_name + '/worlds', ['worlds/simple_obstacle_course.world']),
           ('share/' + package_name + '/launch', ['launch/navigation_simulation_launch.py']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Your Name',
       maintainer_email='your.email@example.com',
       description='Simple navigation simulation lab',
       license='Apache License 2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'navigation_node = lab_navigation_simulation.navigation_node:main',
           ],
       },
   )
   ```

2. **Build the package:**
   ```bash
   cd ~/ros2_sim_ws
   colcon build --packages-select lab_navigation_simulation
   ```

3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

4. **Run the simulation:**
   ```bash
   ros2 launch lab_navigation_simulation navigation_simulation_launch.py
   ```

### Part 7: Experimenting with Navigation Parameters (20 minutes)

1. **Create a parameter file for tuning (`config/navigation_params.yaml`):**
   ```yaml
   navigation_node:
     ros__parameters:
       linear_speed: 0.5
       angular_speed: 0.5
       goal_tolerance: 0.3
       obstacle_threshold: 0.8
       safety_distance: 0.5
   ```

2. **Modify the navigation node to use parameters:**
   ```python
   # Add to __init__ method after creating the node
   self.declare_parameter('linear_speed', 0.5)
   self.declare_parameter('angular_speed', 0.5)
   self.declare_parameter('goal_tolerance', 0.3)
   self.declare_parameter('obstacle_threshold', 0.8)

   # Then use the parameters
   self.linear_speed = self.get_parameter('linear_speed').value
   self.angular_speed = self.get_parameter('angular_speed').value
   self.goal_tolerance = self.get_parameter('goal_tolerance').value
   self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
   ```

3. **Run with custom parameters:**
   ```bash
   ros2 run lab_navigation_simulation navigation_node --ros-args --params-file config/navigation_params.yaml
   ```

### Part 8: Advanced Navigation - Using Navigation2 Stack (30 minutes)

1. **Install Navigation2 if not already installed:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

2. **Create a launch file for Navigation2 (`launch/navigation2_launch.py`):**
   ```python
   import os
   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory


   def generate_launch_description():
       # Launch configuration
       use_sim_time = LaunchConfiguration('use_sim_time', default='true')
       map_yaml_file = LaunchConfiguration('map', default=os.path.join(
           get_package_share_directory('lab_navigation_simulation'),
           'maps',
           'simple_map.yaml'
       ))

       # Package directories
       nav2_bringup_dir = get_package_share_directory('nav2_bringup')
       pkg_share = get_package_share_directory('lab_navigation_simulation')

       # Navigation launch
       navigation_launch = IncludeLaunchDescription(
           PythonLaunchDescriptionSource(
               os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
           ),
           launch_arguments={
               'use_sim_time': use_sim_time,
               'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
           }.items()
       )

       # SLAM toolbox (for mapping)
       slam_toolbox = IncludeLaunchDescription(
           PythonLaunchDescriptionSource(
               os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
           ),
           launch_arguments={
               'use_sim_time': use_sim_time
           }.items()
       )

       return LaunchDescription([
           navigation_launch,
           # slam_toolbox  # Uncomment if you want SLAM
       ])
   ```

### Part 9: Visualization and Monitoring (15 minutes)

1. **Monitor the robot's path using ROS2 tools:**
   ```bash
   # View topics
   ros2 topic list

   # Echo odometry
   ros2 topic echo /odom

   # Echo laser scan
   ros2 topic echo /scan

   # Check robot tf frames
   ros2 run tf2_tools view_frames
   ```

2. **Use RViz for visualization (if available):**
   ```bash
   rviz2
   # In RViz, add displays for:
   # - RobotModel (to see the robot)
   # - LaserScan (to see sensor data)
   # - Path (to see planned paths)
   # - Map (if using SLAM or known map)
   ```

## Discussion Questions

1. How does the simple go-to-goal algorithm compare to more sophisticated path planning methods?

2. What are the advantages and disadvantages of using reactive vs. deliberative navigation approaches?

3. How does the quality of simulation affect the transfer of navigation algorithms to real robots?

4. What sensor data would you need for more robust navigation in complex environments?

5. How would you modify the navigation algorithm to handle dynamic obstacles?

## Extension Activities

1. **Path Planning**: Implement A* or Dijkstra's algorithm for path planning in the grid.

2. **Dynamic Obstacles**: Add moving obstacles to the simulation and implement dynamic obstacle avoidance.

3. **Multi-Goal Navigation**: Create a sequence of waypoints for the robot to visit.

4. **Sensor Fusion**: Combine multiple sensor inputs (LIDAR, camera, IMU) for better navigation.

5. **Performance Metrics**: Implement metrics to evaluate navigation performance (time, path length, collisions).

## Assessment

Complete the following self-assessment:

- I can create custom Gazebo world files: [ ] Yes [ ] No [ ] Partially
- I can spawn robots in Gazebo simulation: [ ] Yes [ ] No [ ] Partially
- I understand basic navigation algorithms: [ ] Yes [ ] No [ ] Partially
- I can implement go-to-goal navigation: [ ] Yes [ ] No [ ] Partially
- I can use ROS2 tools to monitor navigation: [ ] Yes [ ] No [ ] Partially
- I can tune navigation parameters: [ ] Yes [ ] No [ ] Partially

## Troubleshooting Tips

- **Robot not moving**: Check if the cmd_vel topic is connected properly
- **Gazebo not starting**: Verify Gazebo installation and graphics support
- **TF errors**: Ensure all required transforms are published
- **Navigation not working**: Check if odometry and laser scan topics are available
- **Performance issues**: Reduce simulation complexity or adjust physics parameters

## Conclusion

This lab has provided hands-on experience with robot navigation in simulation. You've learned to create custom worlds, implement basic navigation algorithms, and use ROS2 tools for monitoring. Simulation provides a safe environment to test navigation approaches before deploying on real robots.

## Resources for Further Exploration

- Navigation2 Documentation: https://navigation.ros.org/
- Gazebo Tutorials: http://gazebosim.org/tutorials
- ROS2 Navigation Tutorials: http://docs.ros.org/en/humble/Tutorials.html
- Path Planning Algorithms: Research papers and implementations
- Robot Operating System (ROS) documentation
- Open Source Robotics Foundation resources