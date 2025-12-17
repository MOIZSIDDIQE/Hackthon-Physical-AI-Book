# Chapter 2 Lab: Virtual Robot Assembly

## Objective

The goal of this lab is to understand the components of a humanoid robot by virtually assembling one using CAD tools and URDF (Unified Robot Description Format). You will learn how different subsystems connect and interact to create a functional humanoid robot.

## Learning Outcomes

By the end of this lab, you should be able to:

- Identify and describe the major components of a humanoid robot
- Understand how components connect and interact mechanically
- Create a basic robot model using URDF format
- Visualize the assembled robot in a simulation environment
- Analyze the relationship between physical components and robot capabilities

## Materials Needed

- Computer with internet access
- Text editor or IDE
- Webots or Gazebo simulation software (or online URDF viewers)
- Basic knowledge of XML format
- Robot component reference materials

## Background

The Unified Robot Description Format (URDF) is an XML-based format used to represent robot models. It describes the physical and visual properties of a robot, including links (rigid bodies), joints (connections between links), and other properties like inertia, visual representation, and collision properties.

In this lab, you'll build a simplified humanoid robot model with the following components:
- Torso (base link)
- Head with sensors
- Two arms with hands
- Two legs with feet

## Lab Procedure

### Part 1: Understanding URDF Structure (20 minutes)

Before building your robot, familiarize yourself with the basic structure of a URDF file:

```xml
<?xml version="1.0"?>
<robot name="my_humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="joint_name" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

### Part 2: Design Your Robot's Kinematic Structure (30 minutes)

Plan your humanoid robot's structure by identifying:

1. **Base Link**: Where will you start your robot model? (torso, pelvis, or head)
2. **Major Joints**: List the joints you'll need for each body part:
   - Neck joint (head connection)
   - Shoulder joints (2 joints, one for each arm)
   - Elbow joints (2 joints)
   - Wrist joints (2 joints)
   - Hip joints (2 joints)
   - Knee joints (2 joints)
   - Ankle joints (2 joints)

3. **Link Dimensions**: Estimate the size of each body part:
   - Torso: width, depth, height
   - Head: width, depth, height
   - Upper arm: length, diameter
   - Lower arm: length, diameter
   - Hand: length, width, thickness
   - Thigh: length, diameter
   - Lower leg: length, diameter
   - Foot: length, width, thickness

### Part 3: Create Your Robot Model (45 minutes)

Create a URDF file for your humanoid robot. Start with the provided template and customize it:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Torso (base link) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.785" upper="0.785" effort="10" velocity="1"/>
  </joint>

  <!-- Left shoulder -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Continue adding more links and joints for:
       - Right arm
       - Both legs
       - Both feet
       - Include appropriate joints for elbows, knees, wrists, ankles
  -->

  <!-- Right shoulder -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right shoulder joint -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Left hip (for leg) -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Left hip joint -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="0.08 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <!-- Right hip (for leg) -->
  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Right hip joint -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="-0.08 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

</robot>
```

### Part 4: Complete Your Robot Model (30 minutes)

Complete the robot model by adding the remaining components:

1. **Left and right lower arms** (elbows to wrists)
2. **Left and right hands**
3. **Left and right lower legs** (knees to ankles)
4. **Left and right feet**
5. **Joints for elbows, wrists, knees, and ankles**

Here are examples for the elbow joint and lower arm:

```xml
<!-- Left elbow joint -->
<joint name="left_elbow_joint" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="1.57" effort="10" velocity="1"/>
</joint>

<!-- Left lower arm -->
<link name="left_lower_arm">
  <visual>
    <geometry>
      <cylinder length="0.25" radius="0.04"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 0.8"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.25" radius="0.04"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.8"/>
    <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
  </inertial>
</link>
```

### Part 5: Visualization and Testing (25 minutes)

1. **Save your URDF file** with a meaningful name (e.g., `my_humanoid.urdf`)

2. **Visualize your robot** using one of these methods:
   - Use an online URDF viewer
   - Import into Webots or Gazebo if available
   - Use RViz with ROS if available

3. **Test the kinematic structure** by checking:
   - All links are connected properly
   - Joint ranges of motion are reasonable
   - No parts are intersecting when in default position
   - The robot has a stable base

### Part 6: Analysis and Reflection (20 minutes)

Analyze your completed robot model:

1. **Kinematic Chain Analysis**: Trace the path from torso to left hand and right foot. How many joints does each path have?

2. **Degrees of Freedom**: Count the total number of joints in your robot. How many degrees of freedom does this provide?

3. **Stability Considerations**: Based on your model, where is the center of gravity? How might this affect balance?

4. **Functional Assessment**: What movements would your robot be capable of? What limitations do you see?

## Discussion Questions

1. How does the mechanical design of your robot affect its capabilities?

2. What trade-offs did you make when designing joint ranges of motion?

3. How would you modify your design to improve stability?

4. What sensors would you add to your robot model, and where would you place them?

5. How does the physical structure constrain the robot's behavior?

## Extension Activities

1. **Advanced Model**: Add more detailed geometry using mesh files instead of basic shapes.

2. **Sensor Integration**: Add sensor definitions (cameras, IMUs, force sensors) to your URDF.

3. **Gazebo Simulation**: Create a Gazebo world file and spawn your robot in simulation.

4. **Inverse Kinematics**: Research how to control your robot's end effectors using inverse kinematics.

## Assessment

Complete the following self-assessment:

- I understand the basic structure of URDF files: [ ] Yes [ ] No [ ] Partially
- I can identify the major components of a humanoid robot: [ ] Yes [ ] No [ ] Partially
- I can create a simple robot model with proper link-joint relationships: [ ] Yes [ ] No [ ] Partially
- I understand how physical components affect robot capabilities: [ ] Yes [ ] No [ ] Partially
- I can visualize and analyze my robot model: [ ] Yes [ ] No [ ] Partially

## Conclusion

This lab has given you hands-on experience with the mechanical design of humanoid robots. Understanding how components connect and interact is fundamental to creating functional robots. The URDF format is widely used in robotics and provides a standardized way to describe robot models for simulation and control.

## Resources for Further Exploration

- ROS URDF Tutorials: http://wiki.ros.org/urdf/Tutorials
- Gazebo Model Tutorial: http://gazebosim.org/tutorials?tut=build_robot
- Webots Robot Modeling: https://cyberbotics.com/doc/guide/modeling
- Robotics Toolbox: For kinematic analysis of your robot model