# Chapter 3 Lab: Publisher-Subscriber Nodes

## Objective

The goal of this lab is to create and run ROS2 publisher and subscriber nodes that communicate using topics. You will learn how to implement the publish-subscribe communication pattern, create custom messages, and use ROS2 command-line tools to monitor communication.

## Learning Outcomes

By the end of this lab, you should be able to:

- Create ROS2 publisher and subscriber nodes in both C++ and Python
- Understand the publish-subscribe communication pattern
- Use ROS2 command-line tools to monitor and debug communication
- Create and use custom message types
- Launch multiple nodes using launch files
- Configure Quality of Service settings

## Prerequisites

- ROS2 Humble Hawksbill installed and sourced
- Basic knowledge of C++ and Python
- Understanding of ROS2 concepts from Chapter 3
- Text editor or IDE for code development

## Materials Needed

- Computer with ROS2 Humble installed
- Terminal access
- Text editor or IDE
- Basic understanding of ROS2 workspace structure

## Background

The publisher-subscriber pattern is the most common communication mechanism in ROS2. Publishers send messages to topics, and subscribers receive messages from topics. This decoupled communication allows for flexible system design where publishers and subscribers don't need to know about each other.

## Lab Procedure

### Part 1: Setting Up Your Workspace (15 minutes)

1. **Create a new ROS2 workspace:**
   ```bash
   mkdir -p ~/ros2_labs/src
   cd ~/ros2_labs
   ```

2. **Create a new package for this lab:**
   ```bash
   cd src
   ros2 pkg create --build-type ament_cmake lab_publisher_subscriber --dependencies rclcpp rclpy std_msgs
   ```

3. **Navigate to the package directory:**
   ```bash
   cd lab_publisher_subscriber
   ```

### Part 2: Creating a Python Publisher Node (20 minutes)

1. **Create the publisher script directory:**
   ```bash
   mkdir -p lab_publisher_subscriber
   ```

2. **Create the publisher script (`lab_publisher_subscriber/talker.py`):**
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class MinimalPublisher(Node):

       def __init__(self):
           super().__init__('minimal_publisher')
           self.publisher_ = self.create_publisher(String, 'topic', 10)
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = 'Hello World: %d' % self.i
           self.publisher_.publish(msg)
           self.get_logger().info('Publishing: "%s"' % msg.data)
           self.i += 1


   def main(args=None):
       rclpy.init(args=args)

       minimal_publisher = MinimalPublisher()

       rclpy.spin(minimal_publisher)

       # Destroy the node explicitly
       minimal_publisher.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Part 3: Creating a Python Subscriber Node (15 minutes)

1. **Create the subscriber script (`lab_publisher_subscriber/listener.py`):**
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class MinimalSubscriber(Node):

       def __init__(self):
           super().__init__('minimal_subscriber')
           self.subscription = self.create_subscription(
               String,
               'topic',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, msg):
           self.get_logger().info('I heard: "%s"' % msg.data)


   def main(args=None):
       rclpy.init(args=args)

       minimal_subscriber = MinimalSubscriber()

       rclpy.spin(minimal_subscriber)

       # Destroy the node explicitly
       minimal_subscriber.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Part 4: Creating C++ Publisher and Subscriber Nodes (30 minutes)

1. **Create C++ source directory:**
   ```bash
   mkdir -p src
   ```

2. **Create C++ publisher (`src/talker.cpp`):**
   ```cpp
   #include <chrono>
   #include <functional>
   #include <memory>
   #include <string>

   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"

   using namespace std::chrono_literals;

   class MinimalPublisher : public rclcpp::Node
   {
   public:
     MinimalPublisher()
     : Node("minimal_publisher"), count_(0)
     {
       publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
       timer_ = this->create_wall_timer(
         500ms, std::bind(&MinimalPublisher::timer_callback, this));
     }

   private:
     void timer_callback()
     {
       auto message = std_msgs::msg::String();
       message.data = "Hello, world! " + std::to_string(count_++);
       RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
       publisher_->publish(message);
     }
     rclcpp::TimerBase::SharedPtr timer_;
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
     size_t count_;
   };

   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<MinimalPublisher>());
     rclcpp::shutdown();
     return 0;
   }
   ```

3. **Create C++ subscriber (`src/listener.cpp`):**
   ```cpp
   #include <memory>

   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"

   class MinimalSubscriber : public rclcpp::Node
   {
   public:
     MinimalSubscriber()
     : Node("minimal_subscriber")
     {
       subscription_ = this->create_subscription<std_msgs::msg::String>(
         "topic", 10,
         [this](const std_msgs::msg::String::SharedPtr msg) {
           RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
         });
     }

   private:
     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
   };

   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<MinimalSubscriber>());
     rclcpp::shutdown();
     return 0;
   }
   ```

### Part 5: Configuring the Package (20 minutes)

1. **Update the CMakeLists.txt file to include your executables:**
   ```cmake
   cmake_minimum_required(VERSION 3.8)
   project(lab_publisher_subscriber)

   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
     add_compile_options(-Wall -Wextra -Wpedantic)
   endif()

   # find dependencies
   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(rclpy REQUIRED)
   find_package(std_msgs REQUIRED)

   # C++ executables
   add_executable(talker src/talker.cpp)
   ament_target_dependencies(talker rclcpp std_msgs)

   add_executable(listener src/listener.cpp)
   ament_target_dependencies(listener rclcpp std_msgs)

   # Install executables
   install(TARGETS
     talker
     listener
     DESTINATION lib/${PROJECT_NAME}
   )

   # Install Python modules
   ament_python_install_package(${PROJECT_NAME})

   # Install Python executables
   install(PROGRAMS
     lab_publisher_subscriber/talker.py
     lab_publisher_subscriber/listener.py
     DESTINATION lib/${PROJECT_NAME}
   )

   ament_package()
   ```

2. **Update the package.xml file:**
   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>lab_publisher_subscriber</name>
     <version>0.0.0</version>
     <description>Lab for publisher-subscriber nodes</description>
     <maintainer email="user@example.com">User</maintainer>
     <license>Apache-2.0</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <depend>rclcpp</depend>
     <depend>rclpy</depend>
     <depend>std_msgs</depend>

     <test_depend>ament_lint_auto</test_depend>
     <test_depend>ament_lint_common</test_depend>

     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>
   ```

### Part 6: Building and Running the Nodes (20 minutes)

1. **Build the package:**
   ```bash
   cd ~/ros2_labs
   colcon build --packages-select lab_publisher_subscriber
   ```

2. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

3. **Open two terminal windows and run:**
   - Terminal 1: `ros2 run lab_publisher_subscriber talker`
   - Terminal 2: `ros2 run lab_publisher_subscriber listener`

4. **Observe the communication between nodes.**

### Part 7: Using ROS2 Command-Line Tools (15 minutes)

1. **List active topics:**
   ```bash
   ros2 topic list
   ```

2. **Check topic information:**
   ```bash
   ros2 topic info /topic
   ```

3. **Echo messages on the topic:**
   ```bash
   ros2 topic echo /topic
   ```

4. **Get detailed topic information:**
   ```bash
   ros2 interface show std_msgs/msg/String
   ```

5. **Monitor node connections:**
   ```bash
   ros2 node list
   ros2 node info /minimal_publisher
   ros2 node info /minimal_subscriber
   ```

### Part 8: Creating a Launch File (15 minutes)

1. **Create a launch directory:**
   ```bash
   mkdir -p launch
   ```

2. **Create a launch file (`launch/publisher_subscriber_launch.py`):**
   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node


   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='lab_publisher_subscriber',
               executable='talker',
               name='talker_node'
           ),
           Node(
               package='lab_publisher_subscriber',
               executable='listener',
               name='listener_node'
           )
       ])
   ```

3. **Run the launch file:**
   ```bash
   ros2 launch lab_publisher_subscriber publisher_subscriber_launch.py
   ```

### Part 9: Advanced Exercise - Custom Message (25 minutes)

1. **Create a custom message directory:**
   ```bash
   mkdir -p msg
   ```

2. **Create a custom message definition (`msg/CustomMessage.msg`):**
   ```
   string name
   int32 age
   float64 height
   bool is_active
   ```

3. **Update CMakeLists.txt to include custom messages:**
   ```cmake
   find_package(rosidl_default_generators REQUIRED)

   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/CustomMessage.msg"
     DEPENDENCIES std_msgs
   )
   ```

4. **Update package.xml to include message dependencies:**
   ```xml
   <build_depend>rosidl_default_generators</build_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   <member_of_group>rosidl_interface_packages</member_of_group>
   ```

5. **Build the package with the new message:**
   ```bash
   cd ~/ros2_labs
   colcon build --packages-select lab_publisher_subscriber
   source install/setup.bash
   ```

### Part 10: Quality of Service Experimentation (20 minutes)

1. **Modify the Python publisher to use different QoS:**
   ```python
   from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

   class MinimalPublisherQoS(Node):
       def __init__(self):
           super().__init__('minimal_publisher_qos')
           qos_profile = QoSProfile(
               depth=10,
               reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST
           )
           self.publisher_ = self.create_publisher(String, 'qos_topic', qos_profile)
           timer_period = 0.5
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = 'QoS Message: %d' % self.i
           self.publisher_.publish(msg)
           self.get_logger().info('Publishing: "%s"' % msg.data)
           self.i += 1
   ```

2. **Build and run with different QoS settings to observe behavior differences.**

## Discussion Questions

1. How does the publish-subscribe pattern enable loose coupling between nodes?

2. What are the differences between using Python and C++ for ROS2 nodes?

3. How do Quality of Service settings affect communication reliability?

4. What happens if a subscriber starts after a publisher has already been running?

5. How would you modify the nodes to handle message loss or delay?

## Extension Activities

1. **Multiple Publishers/Subscriber**: Create multiple publishers sending to the same topic and observe how the subscriber receives messages.

2. **Message Filtering**: Modify the subscriber to only process messages that meet certain criteria.

3. **Performance Testing**: Measure the latency between publishing and receiving messages.

4. **Node Parameters**: Add parameters to control the publishing rate and message content.

5. **Error Handling**: Add error handling for cases where topics are unavailable.

## Assessment

Complete the following self-assessment:

- I can create ROS2 publisher and subscriber nodes in Python: [ ] Yes [ ] No [ ] Partially
- I can create ROS2 publisher and subscriber nodes in C++: [ ] Yes [ ] No [ ] Partially
- I understand the publish-subscribe communication pattern: [ ] Yes [ ] No [ ] Partially
- I can use ROS2 command-line tools to monitor communication: [ ] Yes [ ] No [ ] Partially
- I can create and use launch files: [ ] Yes [ ] No [ ] Partially
- I understand Quality of Service settings: [ ] Yes [ ] No [ ] Partially

## Troubleshooting Tips

- **Nodes not connecting**: Check that both nodes are on the same ROS domain
- **Permission errors**: Ensure proper permissions on workspace files
- **Build errors**: Verify all dependencies are properly declared
- **Import errors**: Check that the workspace is properly sourced
- **Topic not found**: Verify topic names match between publisher and subscriber

## Conclusion

This lab has provided hands-on experience with the fundamental communication pattern in ROS2. You've learned to create both Python and C++ nodes, use ROS2 tools for monitoring, and organize your code in a proper package structure. The publisher-subscriber pattern is essential for building modular, scalable robotics applications.

## Resources for Further Exploration

- ROS2 Tutorials: http://docs.ros.org/en/humble/Tutorials.html
- ROS2 Design: Understanding ROS2 Architecture
- Quality of Service in ROS2: Advanced configuration options
- ROS2 Launch Files: Complex launch file configurations
- Custom Messages: Advanced message definition and usage