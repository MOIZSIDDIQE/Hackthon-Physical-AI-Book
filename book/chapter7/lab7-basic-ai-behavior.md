# Chapter 7 Lab: Basic AI Behavior for Robotics

## Objective

The goal of this lab is to implement basic AI behaviors for a robot, including simple decision-making, learning from experience, and adaptive behavior. You will create a robot that can navigate, avoid obstacles, and learn to improve its performance over time using basic AI techniques.

## Learning Outcomes

By the end of this lab, you should be able to:

- Implement basic AI decision-making algorithms for robots
- Create simple learning systems that adapt to the environment
- Integrate AI behaviors with robot control systems
- Design state machines for robot behavior management
- Evaluate AI system performance in robotic tasks
- Understand the basics of reinforcement learning in robotics

## Prerequisites

- ROS2 Humble Hawksbill installed
- Basic Python programming skills
- Understanding of robot navigation and control
- Familiarity with basic AI concepts

## Materials Needed

- Computer with ROS2 installed
- Gazebo simulation environment
- Robot simulation environment
- Terminal access and text editor

## Background

Artificial Intelligence in robotics involves creating systems that can perceive their environment, make decisions, and learn from experience. This lab focuses on implementing basic AI behaviors including reactive behaviors, finite state machines, and simple learning algorithms.

## Lab Procedure

### Part 1: Setting Up the Environment (15 minutes)

1. **Create a new ROS2 package for this lab:**
   ```bash
   cd ~/ros2_labs/src
   ros2 pkg create --build-type ament_python lab_basic_ai --dependencies rclpy geometry_msgs nav_msgs sensor_msgs std_msgs
   ```

2. **Navigate to the package directory:**
   ```bash
   cd lab_basic_ai
   ```

3. **Create the main script directory:**
   ```bash
   mkdir -p lab_basic_ai
   ```

### Part 2: Implementing a Simple Behavior Manager (30 minutes)

Create a behavior manager that can switch between different robot behaviors:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

class BehaviorManager(Node):
    def __init__(self):
        super().__init__('behavior_manager')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Robot state
        self.current_position = Point()
        self.current_yaw = 0.0
        self.laser_ranges = []
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')

        # Behavior state
        self.current_behavior = 'wander'  # wander, avoid, seek, follow
        self.behavior_timer = self.create_timer(0.1, self.behavior_loop)

        # Behavior parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.obstacle_threshold = 1.0
        self.target_x = 5.0  # Default target
        self.target_y = 5.0

        self.get_logger().info('Behavior manager initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.laser_ranges = msg.ranges
        self.process_scan_data()

    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )

    def process_scan_data(self):
        """Process laser scan to detect obstacles"""
        if not self.laser_ranges:
            return

        # Get front-facing ranges (narrow beam in front)
        front_ranges = self.laser_ranges[len(self.laser_ranges)//2-10:len(self.laser_ranges)//2+10]
        front_ranges = [r for r in front_ranges if not math.isnan(r) and r > 0]

        if front_ranges:
            self.obstacle_distance = min(front_ranges)
            self.obstacle_detected = self.obstacle_distance < self.obstacle_threshold
        else:
            self.obstacle_detected = False
            self.obstacle_distance = float('inf')

    def behavior_loop(self):
        """Main behavior selection and execution loop"""
        # Select behavior based on state
        if self.obstacle_detected:
            self.current_behavior = 'avoid'
        else:
            self.current_behavior = 'wander'  # Could be 'seek', 'follow', etc.

        # Execute selected behavior
        cmd_vel = self.execute_behavior(self.current_behavior)
        self.cmd_vel_pub.publish(cmd_vel)

    def execute_behavior(self, behavior):
        """Execute the specified behavior"""
        cmd = Twist()

        if behavior == 'wander':
            cmd = self.wander_behavior()
        elif behavior == 'avoid':
            cmd = self.avoid_behavior()
        elif behavior == 'seek':
            cmd = self.seek_behavior()
        elif behavior == 'follow':
            cmd = self.follow_behavior()

        return cmd

    def wander_behavior(self):
        """Random wandering behavior"""
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = (np.random.random() - 0.5) * self.angular_speed * 0.5  # Gentle turns
        return cmd

    def avoid_behavior(self):
        """Obstacle avoidance behavior"""
        cmd = Twist()

        if self.obstacle_distance < 0.5:  # Very close
            cmd.linear.x = -0.2  # Back up slightly
            cmd.angular.z = self.angular_speed  # Turn significantly
        else:
            cmd.linear.x = 0.0  # Stop forward motion
            cmd.angular.z = self.angular_speed  # Turn away

        return cmd

    def seek_behavior(self):
        """Go to target position behavior"""
        cmd = Twist()

        # Calculate angle to target
        dx = self.target_x - self.current_position.x
        dy = self.target_y - self.current_position.y
        target_angle = math.atan2(dy, dx)

        # Calculate angle difference
        angle_diff = target_angle - self.current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Simple proportional controller
        if abs(angle_diff) > 0.1:
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 2 * angle_diff))
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        return cmd

    def follow_behavior(self):
        """Follow a wall or obstacle"""
        cmd = Twist()

        # Simple wall following using laser ranges
        if len(self.laser_ranges) > 20:
            left_range = np.mean(self.laser_ranges[:10]) if all(r > 0 and not math.isnan(r) for r in self.laser_ranges[:10]) else float('inf')
            front_range = np.mean(self.laser_ranges[len(self.laser_ranges)//2-5:len(self.laser_ranges)//2+5]) if all(r > 0 and not math.isnan(r) for r in self.laser_ranges[len(self.laser_ranges)//2-5:len(self.laser_ranges)//2+5]) else float('inf')
            right_range = np.mean(self.laser_ranges[-10:]) if all(r > 0 and not math.isnan(r) for r in self.laser_ranges[-10:]) else float('inf')

            # Simple wall following algorithm
            desired_distance = 1.0
            error = desired_distance - right_range

            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.5 * error  # Proportional control

        return cmd
```

### Part 3: Implementing a Finite State Machine (30 minutes)

Create a more sophisticated state machine for robot behavior:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import numpy as np

class RobotStateMachine(Node):
    def __init__(self):
        super().__init__('robot_state_machine')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.state_pub = self.create_publisher(String, 'robot_state', 10)

        # Robot state
        self.laser_ranges = []
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.battery_level = 100.0  # Simulated battery
        self.task_complete = False

        # State machine
        self.current_state = 'SEARCHING'  # SEARCHING, AVOIDING, CHARGING, TASK_COMPLETE
        self.state_timer = self.create_timer(0.1, self.state_machine_loop)

        # State parameters
        self.obstacle_threshold = 0.8
        self.charging_distance = 0.5
        self.battery_threshold = 20.0

        self.get_logger().info('Robot state machine initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.laser_ranges = msg.ranges
        self.process_scan_for_obstacles()

    def process_scan_for_obstacles(self):
        """Process scan data to detect obstacles"""
        if not self.laser_ranges:
            return

        # Check for obstacles in front
        front_section = len(self.laser_ranges) // 4
        front_ranges = self.laser_ranges[len(self.laser_ranges)//2 - front_section//2 : len(self.laser_ranges)//2 + front_section//2]
        front_ranges = [r for r in front_ranges if not math.isnan(r) and r > 0]

        if front_ranges:
            self.obstacle_distance = min(front_ranges)
            self.obstacle_detected = self.obstacle_distance < self.obstacle_threshold
        else:
            self.obstacle_detected = False
            self.obstacle_distance = float('inf')

    def state_machine_loop(self):
        """Main state machine execution loop"""
        # Update battery (simulated consumption)
        self.battery_level = max(0.0, self.battery_level - 0.01)

        # Execute current state
        cmd_vel = self.execute_current_state()

        # Check for state transitions
        next_state = self.determine_next_state()
        if next_state != self.current_state:
            self.get_logger().info(f'State transition: {self.current_state} -> {next_state}')
            self.current_state = next_state

        # Publish state
        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)

        # Publish command
        if cmd_vel:
            self.cmd_vel_pub.publish(cmd_vel)

    def execute_current_state(self):
        """Execute the current state's behavior"""
        if self.current_state == 'SEARCHING':
            return self.searching_behavior()
        elif self.current_state == 'AVOIDING':
            return self.avoiding_behavior()
        elif self.current_state == 'CHARGING':
            return self.charging_behavior()
        elif self.current_state == 'TASK_COMPLETE':
            return self.task_complete_behavior()
        else:
            return Twist()  # Stop if unknown state

    def determine_next_state(self):
        """Determine the next state based on conditions"""
        if self.battery_level < self.battery_threshold:
            return 'CHARGING'
        elif self.task_complete:
            return 'TASK_COMPLETE'
        elif self.obstacle_detected:
            return 'AVOIDING'
        else:
            return 'SEARCHING'

    def searching_behavior(self):
        """Search for targets or explore environment"""
        cmd = Twist()
        cmd.linear.x = 0.4  # Forward motion
        cmd.angular.z = (np.random.random() - 0.5) * 0.3  # Gentle random turns
        return cmd

    def avoiding_behavior(self):
        """Avoid detected obstacles"""
        cmd = Twist()

        if self.obstacle_distance < 0.3:
            cmd.linear.x = -0.2  # Back up
            cmd.angular.z = 0.8  # Sharp turn
        elif self.obstacle_distance < 0.6:
            cmd.linear.x = 0.0  # Stop
            cmd.angular.z = 0.6  # Turn
        else:
            cmd.linear.x = 0.3  # Slow forward
            cmd.angular.z = 0.2  # Gentle turn

        return cmd

    def charging_behavior(self):
        """Navigate to charging station"""
        cmd = Twist()

        # Simulate finding charging station
        # In real implementation, this would use localization to find charging station
        if np.random.random() > 0.95:  # 5% chance per cycle to "find" charger
            self.get_logger().info('Charging station found!')
            self.battery_level = min(100.0, self.battery_level + 2.0)  # Simulate charging

        # Look for charging station
        cmd.linear.x = 0.1  # Slow forward
        cmd.angular.z = 0.5  # Turn to scan
        return cmd

    def task_complete_behavior(self):
        """Behavior when task is complete"""
        cmd = Twist()
        # Robot stops and waits for new task
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd
```

### Part 4: Implementing Simple Learning Algorithm (45 minutes)

Create a Q-learning implementation for simple navigation learning:

```python
#!/usr/bin/env python3
import numpy as np
import random
from collections import defaultdict
import json

class QLearningAgent:
    def __init__(self, state_size, action_size, learning_rate=0.1, discount_factor=0.95, epsilon=0.1):
        self.state_size = state_size
        self.action_size = action_size
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor
        self.epsilon = epsilon

        # Q-table: state -> [Q(s, a0), Q(s, a1), ...]
        self.q_table = defaultdict(lambda: np.zeros(action_size))

    def discretize_state(self, continuous_state):
        """
        Convert continuous state to discrete state representation
        For this example, we'll discretize distance and angle
        """
        distance, angle = continuous_state

        # Discretize distance into bins
        distance_bins = 5
        angle_bins = 8

        distance_idx = min(int(distance * distance_bins), distance_bins - 1)
        angle_idx = int((angle + np.pi) * angle_bins / (2 * np.pi)) % angle_bins

        return (distance_idx, angle_idx)

    def get_action(self, state):
        """
        Choose action using epsilon-greedy policy
        """
        discrete_state = self.discretize_state(state)

        if random.random() < self.epsilon:
            # Explore: choose random action
            return random.choice(range(self.action_size))
        else:
            # Exploit: choose best action based on Q-table
            q_values = self.q_table[discrete_state]
            return np.argmax(q_values)

    def update_q_value(self, state, action, reward, next_state):
        """
        Update Q-value using Q-learning formula
        """
        current_state = self.discretize_state(state)
        next_discrete_state = self.discretize_state(next_state)

        # Get current Q-value
        current_q = self.q_table[current_state][action]

        # Get maximum Q-value for next state
        next_max_q = np.max(self.q_table[next_discrete_state])

        # Calculate new Q-value
        new_q = current_q + self.learning_rate * (
            reward + self.discount_factor * next_max_q - current_q
        )

        # Update Q-table
        self.q_table[current_state][action] = new_q

    def save_q_table(self, filename):
        """Save Q-table to file"""
        # Convert defaultdict to regular dict for JSON serialization
        q_table_dict = {str(k): v.tolist() for k, v in self.q_table.items()}
        with open(filename, 'w') as f:
            json.dump(q_table_dict, f)

    def load_q_table(self, filename):
        """Load Q-table from file"""
        with open(filename, 'r') as f:
            q_table_dict = json.load(f)

        # Convert back to defaultdict
        for k, v in q_table_dict.items():
            # Convert string key back to tuple
            state_key = eval(k)  # Note: eval is generally unsafe, but acceptable here for controlled data
            self.q_table[state_key] = np.array(v)

class NavigationLearning:
    def __init__(self):
        # Define action space: [forward, turn_left, turn_right, backward]
        self.action_space = 4
        self.action_descriptions = {
            0: "move_forward",
            1: "turn_left",
            2: "turn_right",
            3: "move_backward"
        }

        # Initialize Q-learning agent
        self.agent = QLearningAgent(
            state_size=2,  # distance, angle
            action_size=self.action_space
        )

        # Learning parameters
        self.episode_count = 0
        self.max_episodes = 1000
        self.current_step = 0
        self.max_steps_per_episode = 100

        # Navigation parameters
        self.goal_distance_threshold = 0.5
        self.obstacle_threshold = 0.8

    def calculate_reward(self, distance_to_goal, obstacle_distance, action_taken):
        """
        Calculate reward based on current situation
        """
        reward = 0.0

        # Distance to goal reward (closer is better)
        if distance_to_goal < self.goal_distance_threshold:
            reward += 10.0  # Reached goal
        else:
            reward -= distance_to_goal * 0.1  # Penalty for being far from goal

        # Obstacle avoidance penalty
        if obstacle_distance < self.obstacle_threshold:
            reward -= 5.0  # Significant penalty for being too close to obstacles

        # Action-based rewards/penalties
        if action_taken == 0:  # Moving forward
            reward += 0.1  # Small reward for forward progress
        elif action_taken in [1, 2]:  # Turning
            reward -= 0.05  # Small penalty for turning (inefficient)

        return reward

    def get_next_state(self, current_distance, current_angle, action_taken):
        """
        Simulate next state based on action (simplified physics)
        """
        # Simplified state transition
        # In reality, this would involve actual robot physics and sensor feedback
        new_distance = current_distance
        new_angle = current_angle

        if action_taken == 0:  # Move forward
            new_distance = max(0.1, current_distance - 0.1)  # Get closer to goal
        elif action_taken == 1:  # Turn left
            new_angle = (current_angle + 0.2) % (2 * np.pi)
        elif action_taken == 2:  # Turn right
            new_angle = (current_angle - 0.2) % (2 * np.pi)
        elif action_taken == 3:  # Move backward
            new_distance = min(5.0, current_distance + 0.1)  # Get further from goal

        # Add some noise to make it more realistic
        new_distance += np.random.normal(0, 0.05)
        new_angle += np.random.normal(0, 0.05)

        return (new_distance, new_angle)

    def train_episode(self, initial_distance=3.0, initial_angle=0.0):
        """
        Run one training episode
        """
        state = (initial_distance, initial_angle)
        total_reward = 0.0
        steps = 0

        while steps < self.max_steps_per_episode:
            # Choose action
            action = self.agent.get_action(state)

            # Simulate environment (get next state and reward)
            next_state = self.get_next_state(state[0], state[1], action)
            reward = self.calculate_reward(
                next_state[0],  # distance to goal
                2.0,  # obstacle distance (simulated)
                action
            )

            # Update Q-value
            self.agent.update_q_value(state, action, reward, next_state)

            # Update state and accumulate reward
            state = next_state
            total_reward += reward
            steps += 1

            # Check if goal reached
            if state[0] < self.goal_distance_threshold:
                break

        self.episode_count += 1
        return total_reward, steps

    def learn_navigation(self):
        """
        Main learning loop
        """
        rewards_history = []

        for episode in range(self.max_episodes):
            initial_distance = np.random.uniform(1.0, 4.0)
            initial_angle = np.random.uniform(0, 2 * np.pi)

            total_reward, steps = self.train_episode(initial_distance, initial_angle)
            rewards_history.append(total_reward)

            # Decay epsilon for more exploitation over time
            self.agent.epsilon = max(0.01, self.agent.epsilon * 0.995)

            if episode % 100 == 0:
                avg_reward = np.mean(rewards_history[-100:]) if len(rewards_history) >= 100 else np.mean(rewards_history)
                print(f"Episode {episode}, Avg Reward: {avg_reward:.2f}, Epsilon: {self.agent.epsilon:.3f}")

        return rewards_history

    def get_optimal_action(self, distance, angle):
        """
        Get the best action for given state (after training)
        """
        state = (distance, angle)
        discrete_state = self.agent.discretize_state(state)
        q_values = self.agent.q_table[discrete_state]
        return np.argmax(q_values)
```

### Part 5: Creating the Main AI Controller Node (35 minutes)

Create the main node that integrates all AI components:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
import math
import numpy as np
from lab_basic_ai.navigation_learning import NavigationLearning

class AIBehaviorController(Node):
    def __init__(self):
        super().__init__('ai_behavior_controller')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.behavior_state_pub = self.create_publisher(String, 'ai_behavior_state', 10)
        self.performance_pub = self.create_publisher(Float32, 'ai_performance', 10)

        # Robot state
        self.laser_ranges = []
        self.current_position = [0.0, 0.0]
        self.current_yaw = 0.0
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')

        # AI components
        self.behavior_mode = 'reactive'  # 'reactive', 'learning', 'state_machine'
        self.learning_agent = NavigationLearning()

        # Reactive behavior parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.obstacle_threshold = 0.8

        # Learning parameters
        self.learning_enabled = False
        self.performance_score = 0.0

        # Control timer
        self.control_timer = self.create_timer(0.1, self.ai_control_loop)

        self.get_logger().info('AI Behavior Controller initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.laser_ranges = msg.ranges
        self.process_scan_data()

    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y

        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )

    def process_scan_data(self):
        """Process laser scan to detect obstacles"""
        if not self.laser_ranges:
            return

        # Get front-facing ranges
        front_section = len(self.laser_ranges) // 8
        front_ranges = self.laser_ranges[len(self.laser_ranges)//2 - front_section//2 : len(self.laser_ranges)//2 + front_section//2]
        front_ranges = [r for r in front_ranges if not math.isnan(r) and r > 0]

        if front_ranges:
            self.obstacle_distance = min(front_ranges)
            self.obstacle_detected = self.obstacle_distance < self.obstacle_threshold
        else:
            self.obstacle_detected = False
            self.obstacle_distance = float('inf')

    def ai_control_loop(self):
        """Main AI control loop"""
        if self.behavior_mode == 'reactive':
            cmd_vel = self.reactive_behavior()
        elif self.behavior_mode == 'learning':
            cmd_vel = self.learning_behavior()
        elif self.behavior_mode == 'state_machine':
            cmd_vel = self.state_machine_behavior()
        else:
            cmd_vel = Twist()  # Stop if unknown mode

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

        # Publish behavior state
        state_msg = String()
        state_msg.data = self.behavior_mode
        self.behavior_state_pub.publish(state_msg)

        # Publish performance (simplified)
        perf_msg = Float32()
        perf_msg.data = self.performance_score
        self.performance_pub.publish(perf_msg)

    def reactive_behavior(self):
        """Simple reactive behavior"""
        cmd = Twist()

        if self.obstacle_detected:
            # Obstacle avoidance
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
        else:
            # Forward motion
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        return cmd

    def learning_behavior(self):
        """Behavior using learned policy"""
        if not self.learning_enabled:
            return self.reactive_behavior()

        # Create state representation (simplified)
        # In real implementation, this would be more sophisticated
        distance_state = min(5.0, self.obstacle_distance) / 5.0  # Normalize
        angle_state = (self.current_yaw + math.pi) / (2 * math.pi)  # Normalize

        # Use learning agent to choose action
        # This is a simplified example - real implementation would be more complex
        if self.obstacle_detected:
            # If obstacle detected, turn away
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
        else:
            # Otherwise, move forward
            cmd = Twist()
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        return cmd

    def state_machine_behavior(self):
        """Behavior based on finite state machine"""
        # This would integrate with the state machine from Part 3
        # For this implementation, we'll use a simplified version
        cmd = Twist()

        if self.obstacle_detected:
            # Avoid obstacle
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
        else:
            # Move forward
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        return cmd

    def toggle_learning_mode(self):
        """Toggle between learning and reactive modes"""
        if self.behavior_mode == 'reactive':
            self.behavior_mode = 'learning'
            self.learning_enabled = True
            self.get_logger().info('Switched to learning mode')
        else:
            self.behavior_mode = 'reactive'
            self.learning_enabled = False
            self.get_logger().info('Switched to reactive mode')

def main(args=None):
    rclpy.init(args=args)
    ai_controller = AIBehaviorController()

    try:
        rclpy.spin(ai_controller)
    except KeyboardInterrupt:
        pass
    finally:
        ai_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Part 6: Creating a Learning Trainer Node (25 minutes)

Create a node to train the learning agent:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from lab_basic_ai.navigation_learning import NavigationLearning

class LearningTrainer(Node):
    def __init__(self):
        super().__init__('learning_trainer')

        self.trainer_pub = self.create_publisher(String, 'training_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'training_commands', 10)

        # Initialize learning agent
        self.learning_agent = NavigationLearning()

        # Training parameters
        self.training_active = False
        self.episodes_completed = 0
        self.max_training_episodes = 500

        # Training timer
        self.training_timer = None

        self.get_logger().info('Learning trainer initialized')

    def start_training(self):
        """Start the training process"""
        self.training_active = True
        self.get_logger().info(f'Starting training for {self.max_training_episodes} episodes')

        # Run training in a background task
        self.training_timer = self.create_timer(0.1, self.training_step)

    def training_step(self):
        """Perform one training step"""
        if self.episodes_completed >= self.max_training_episodes:
            self.stop_training()
            return

        # Train one episode
        initial_distance = 3.0  # Fixed for consistency
        initial_angle = 0.0

        total_reward, steps = self.learning_agent.train_episode(initial_distance, initial_angle)

        self.episodes_completed += 1

        # Log progress
        if self.episodes_completed % 50 == 0:
            self.get_logger().info(f'Training progress: {self.episodes_completed}/{self.max_training_episodes}, Reward: {total_reward:.2f}')

    def stop_training(self):
        """Stop the training process"""
        self.training_active = False
        if self.training_timer:
            self.training_timer.cancel()

        # Save the trained model
        self.learning_agent.agent.save_q_table('trained_navigation_model.json')
        self.get_logger().info(f'Training completed after {self.episodes_completed} episodes')

        # Publish completion status
        status_msg = String()
        status_msg.data = f'Training completed after {self.episodes_completed} episodes'
        self.trainer_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    trainer = LearningTrainer()

    try:
        # Start training immediately
        trainer.start_training()

        rclpy.spin(trainer)
    except KeyboardInterrupt:
        trainer.stop_training()
    finally:
        trainer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Part 7: Creating a Behavior Tester Node (20 minutes)

Create a node to test different behaviors:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
import time

class BehaviorTester(Node):
    def __init__(self):
        super().__init__('behavior_tester')

        self.mode_pub = self.create_publisher(String, 'ai_behavior_mode', 10)
        self.performance_pub = self.create_publisher(Int32, 'behavior_performance', 10)

        # Test parameters
        self.test_modes = ['reactive', 'learning', 'state_machine']
        self.current_test_index = 0
        self.test_duration = 30  # seconds per test
        self.start_time = time.time()

        # Test timer
        self.test_timer = self.create_timer(1.0, self.run_test)

        self.get_logger().info('Behavior tester initialized')

    def run_test(self):
        """Run behavior tests"""
        current_time = time.time()

        if current_time - self.start_time >= self.test_duration:
            # Move to next test mode
            self.current_test_index = (self.current_test_index + 1) % len(self.test_modes)
            self.start_time = current_time

        # Publish current test mode
        mode_msg = String()
        mode_msg.data = self.test_modes[self.current_test_index]
        self.mode_pub.publish(mode_msg)

        # Publish performance metric (simulated)
        perf_msg = Int32()
        perf_msg.data = int((current_time - self.start_time) * 10) % 100  # Simulated performance
        self.performance_pub.publish(perf_msg)

        self.get_logger().info(f'Running test: {self.test_modes[self.current_test_index]}, '
                              f'Time remaining: {self.test_duration - int(current_time - self.start_time)}s')

def main(args=None):
    rclpy.init(args=args)
    tester = BehaviorTester()

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

### Part 8: Creating Launch File (10 minutes)

Create a launch file to run the complete AI system:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab_basic_ai',
            executable='ai_behavior_controller',
            name='ai_behavior_controller',
            parameters=[
                {'behavior_mode': 'reactive'},
                {'learning_enabled': False}
            ],
            output='screen'
        ),
        Node(
            package='lab_basic_ai',
            executable='learning_trainer',
            name='learning_trainer',
            output='screen'
        ),
        Node(
            package='lab_basic_ai',
            executable='behavior_tester',
            name='behavior_tester',
            output='screen'
        )
    ])
```

### Part 9: Updating setup.py and Building (15 minutes)

Update the setup.py file:

```python
from setuptools import setup

package_name = 'lab_basic_ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ai_behavior_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Basic AI behaviors for robotics lab',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ai_behavior_controller = lab_basic_ai.ai_controller:main',
            'learning_trainer = lab_basic_ai.learning_trainer:main',
            'behavior_tester = lab_basic_ai.behavior_tester:main',
        ],
    },
)
```

Build the package:

```bash
cd ~/ros2_labs
colcon build --packages-select lab_basic_ai
source install/setup.bash
```

## Discussion Questions

1. How do reactive behaviors compare to learning-based behaviors in terms of adaptability and robustness?

2. What are the advantages and disadvantages of finite state machines for robot behavior?

3. How does the exploration-exploitation trade-off affect learning in robotic systems?

4. What factors should be considered when choosing between different AI approaches for robotics?

5. How can safety be ensured when implementing learning algorithms on physical robots?

## Extension Activities

1. **Advanced Learning**: Implement Deep Q-Network (DQN) for more complex learning
2. **Multi-Agent Systems**: Extend to multiple robots learning and cooperating
3. **Imitation Learning**: Train robot behaviors from human demonstrations
4. **Path Planning Integration**: Combine AI behaviors with advanced path planning
5. **Real Robot Deployment**: Deploy learned behaviors on a physical robot

## Assessment

Complete the following self-assessment:

- I can implement reactive robot behaviors: [ ] Yes [ ] No [ ] Partially
- I understand finite state machines for robotics: [ ] Yes [ ] No [ ] Partially
- I can implement basic learning algorithms: [ ] Yes [ ] No [ ] Partially
- I can integrate AI with ROS2 systems: [ ] Yes [ ] No [ ] Partially
- I understand the trade-offs between different AI approaches: [ ] Yes [ ] No [ ] Partially

## Troubleshooting Tips

- **Performance Issues**: Reduce complexity of learning algorithms for real-time applications
- **State Space**: Keep state representation manageable for learning algorithms
- **Reward Design**: Carefully design reward functions to achieve desired behavior
- **Exploration**: Balance exploration and exploitation appropriately
- **Safety**: Always include safety constraints in AI behavior design

## Conclusion

This lab has provided hands-on experience with implementing basic AI behaviors for robotics, including reactive behaviors, state machines, and simple learning algorithms. You've learned to integrate these approaches with ROS2 and understand the trade-offs between different AI techniques.

## Resources for Further Exploration

- "Reinforcement Learning: An Introduction" by Sutton and Barto
- ROS2 Navigation and Planning documentation
- OpenAI Gym for reinforcement learning environments
- Research papers on AI for robotics
- Deep learning frameworks for robotics applications
- Behavior trees and hierarchical task networks for robotics