---
title: AI and Perception for Robotics
sidebar_position: 6
---

# AI and Perception for Robotics

Welcome to the fascinating intersection of artificial intelligence and robotics! In this chapter, we'll explore how robots perceive their environment, make intelligent decisions, and interact with the world around them. This is where robotics meets AI, creating truly intelligent machines capable of autonomous operation.

## What You Will Learn

In this chapter, you'll:

- Understand the fundamentals of robot perception systems
- Learn about computer vision and object recognition for robots
- Master sensor fusion techniques for robust perception
- Implement machine learning algorithms for robotics
- Explore path planning with AI techniques
- Understand human-robot interaction using AI
- Work with deep learning for robotic perception
- Create intelligent robot behaviors using AI

By the end of this chapter, you'll have the knowledge to implement AI-powered perception and decision-making systems for robots.

## Robot Perception Systems

Robot perception is the process by which robots acquire and interpret information about their environment. Unlike humans who have evolved sophisticated sensory systems, robots must rely on artificial sensors and computational algorithms to understand their world.

### Types of Perception:
- **Proprioceptive**: Internal sensors measuring robot state
- **Exteroceptive**: External sensors measuring the environment
- **Interoceptive**: Sensors measuring internal robot conditions

## Computer Vision for Robotics

Computer vision is crucial for robots to "see" and understand their environment. Modern robots use cameras combined with AI algorithms to recognize objects, navigate spaces, and interact with humans.

### Key Computer Vision Tasks:
- **Object detection**: Identifying objects in images
- **Object recognition**: Classifying detected objects
- **Pose estimation**: Determining object position and orientation
- **Scene understanding**: Interpreting the overall scene
- **Visual tracking**: Following objects over time

### OpenCV with ROS2 Example:
```python
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Load pre-trained model (YOLO example)
        self.net = cv2.dnn.readNet('yolov4.weights', 'yolov4.cfg')
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Perform object detection
        height, width, channels = cv_image.shape
        blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        # Process detections
        class_ids = []
        confidences = []
        boxes = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > 0.5:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply non-maximum suppression
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        # Draw bounding boxes
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(class_ids[i])
                confidence = confidences[i]
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(cv_image, f'{label} {confidence:.2f}',
                           (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the image
        cv2.imshow('Object Detection', cv_image)
        cv2.waitKey(1)
```

## Sensor Fusion

Robots rarely rely on a single sensor. Sensor fusion combines data from multiple sensors to create a more accurate and robust understanding of the environment.

### Common Sensor Types:
- **Cameras**: Visual information
- **LIDAR**: 3D spatial information
- **IMU**: Orientation and acceleration
- **GPS**: Position (outdoor)
- **Sonar/Ultrasonic**: Distance measurement
- **Force/Torque**: Physical interaction

### Kalman Filter Example:
```python
import numpy as np

class KalmanFilter:
    def __init__(self, dt, process_noise, measurement_noise):
        self.dt = dt

        # State vector [position, velocity]
        self.x = np.array([[0.0], [0.0]])

        # State transition matrix
        self.F = np.array([[1, dt],
                          [0, 1]])

        # Measurement matrix
        self.H = np.array([[1, 0]])

        # Process noise covariance
        self.Q = np.array([[dt**4/4, dt**3/2],
                          [dt**3/2, dt**2]])
        self.Q *= process_noise

        # Measurement noise covariance
        self.R = np.array([[measurement_noise]])

        # Error covariance
        self.P = np.eye(2)

    def predict(self):
        """Prediction step"""
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x[0, 0]

    def update(self, measurement):
        """Update step with measurement"""
        # Calculate Kalman gain
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # Update state estimate
        y = measurement - np.dot(self.H, self.x)  # Innovation
        self.x = self.x + np.dot(K, y)

        # Update error covariance
        I = np.eye(len(K))
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

        return self.x[0, 0]

# Example usage
kf = KalmanFilter(dt=0.1, process_noise=0.1, measurement_noise=0.5)
position_estimate = kf.predict()
position_estimate = kf.update(measurement=10.5)  # New sensor reading
```

## Machine Learning for Robotics

Machine learning enables robots to learn from experience and adapt to new situations. There are several ML approaches used in robotics:

### Supervised Learning
Used for tasks like object recognition, where the robot learns from labeled examples.

### Reinforcement Learning
Used for learning control policies through trial and error with rewards.

### Unsupervised Learning
Used for clustering similar experiences or discovering patterns in sensor data.

### Deep Learning Example:
```python
import torch
import torch.nn as nn
import torch.optim as optim

class RobotVisionNet(nn.Module):
    def __init__(self, num_classes=10):
        super(RobotVisionNet, self).__init__()
        # Convolutional layers for feature extraction
        self.features = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2)
        )

        # Classifier
        self.classifier = nn.Sequential(
            nn.Dropout(),
            nn.Linear(128 * 28 * 28, 512),  # Adjust based on input size
            nn.ReLU(inplace=True),
            nn.Dropout(),
            nn.Linear(512, num_classes),
        )

    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), -1)
        x = self.classifier(x)
        return x

# Training example
def train_robot_vision(model, train_loader, epochs=10):
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    for epoch in range(epochs):
        running_loss = 0.0
        for i, (inputs, labels) in enumerate(train_loader):
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()

            running_loss += loss.item()
            if i % 100 == 99:  # Print every 100 mini-batches
                print(f'Epoch {epoch + 1}, Batch {i + 1}, Loss: {running_loss / 100:.4f}')
                running_loss = 0.0
```

## Path Planning with AI

AI techniques enhance traditional path planning by enabling robots to adapt to dynamic environments and learn from experience.

### A* with Learning:
```python
import heapq

class AdaptiveAStar:
    def __init__(self, grid):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0])
        self.knowledge = {}  # Learned costs

    def heuristic(self, a, b):
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, pos):
        """Get valid neighbors"""
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            x, y = pos[0] + dx, pos[1] + dy
            if 0 <= x < self.width and 0 <= y < self.height and self.grid[y][x] == 0:
                neighbors.append((x, y))
        return neighbors

    def plan_path(self, start, goal):
        """Plan path using A* with learned knowledge"""
        frontier = [(0, start)]
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal:
                break

            for next_pos in self.get_neighbors(current):
                # Get base cost (learned or default)
                base_cost = self.knowledge.get((current, next_pos), 1)
                new_cost = cost_so_far[current] + base_cost

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal, next_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        # Reconstruct path
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()

        return path

    def update_knowledge(self, path, actual_costs):
        """Update learned costs based on experience"""
        for i in range(len(path) - 1):
            edge = (path[i], path[i + 1])
            # Simple learning rule: average of old and new cost
            old_cost = self.knowledge.get(edge, 1)
            new_cost = actual_costs[i]
            learned_cost = 0.7 * old_cost + 0.3 * new_cost
            self.knowledge[edge] = learned_cost
```

## Human-Robot Interaction (HRI) with AI

Modern robots use AI to understand and respond to human commands, emotions, and intentions.

### Natural Language Processing:
```python
import speech_recognition as sr
import pyttsx3
import nltk
from nltk.tokenize import word_tokenize
from nltk.tag import pos_tag

class RobotNLU:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.tts_engine = pyttsx3.init()

        # Set up voice properties
        voices = self.tts_engine.getProperty('voices')
        self.tts_engine.setProperty('rate', 150)
        self.tts_engine.setProperty('volume', 0.9)

    def listen(self):
        """Listen to user speech and convert to text"""
        with self.microphone as source:
            print("Listening...")
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)

        try:
            text = self.recognizer.recognize_google(audio)
            print(f"Recognized: {text}")
            return text
        except sr.UnknownValueError:
            print("Could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"Error: {e}")
            return None

    def process_command(self, text):
        """Process natural language command"""
        if not text:
            return None

        tokens = word_tokenize(text.lower())
        pos_tags = pos_tag(tokens)

        # Extract action and object
        action = None
        obj = None

        for word, pos in pos_tags:
            if pos.startswith('VB'):  # Verb
                action = word
            elif pos.startswith('NN'):  # Noun
                obj = word

        return {
            'action': action,
            'object': obj,
            'raw_text': text
        }

    def speak(self, text):
        """Speak text to user"""
        print(f"Robot says: {text}")
        self.tts_engine.say(text)
        self.tts_engine.runAndWait()

# Example usage
robot_nlu = RobotNLU()
command = robot_nlu.listen()
parsed = robot_nlu.process_command(command)
if parsed:
    robot_nlu.speak(f"I understood you want to {parsed['action']} the {parsed['object']}")
```

## Deep Learning for Robot Control

Deep reinforcement learning enables robots to learn complex behaviors through trial and error:

### Deep Q-Network (DQN) for Robot Control:
```python
import torch
import torch.nn as nn
import torch.optim as optim
import random
import numpy as np
from collections import deque

class DQN(nn.Module):
    def __init__(self, state_size, action_size, hidden_size=64):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, action_size)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)

class RobotDQNAgent:
    def __init__(self, state_size, action_size, lr=0.001):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=10000)
        self.epsilon = 1.0  # Exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = lr

        # Neural networks
        self.q_network = DQN(state_size, action_size)
        self.target_network = DQN(state_size, action_size)
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=lr)

        # Update target network
        self.update_target_network()

    def update_target_network(self):
        """Copy weights from main network to target network"""
        self.target_network.load_state_dict(self.q_network.state_dict())

    def remember(self, state, action, reward, next_state, done):
        """Store experience in replay memory"""
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        """Choose action using epsilon-greedy policy"""
        if np.random.random() <= self.epsilon:
            return random.randrange(self.action_size)

        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        q_values = self.q_network(state_tensor)
        return np.argmax(q_values.cpu().data.numpy())

    def replay(self, batch_size=32):
        """Train on random batch from memory"""
        if len(self.memory) < batch_size:
            return

        batch = random.sample(self.memory, batch_size)
        states = torch.FloatTensor([e[0] for e in batch])
        actions = torch.LongTensor([e[1] for e in batch])
        rewards = torch.FloatTensor([e[2] for e in batch])
        next_states = torch.FloatTensor([e[3] for e in batch])
        dones = torch.BoolTensor([e[4] for e in batch])

        current_q_values = self.q_network(states).gather(1, actions.unsqueeze(1))
        next_q_values = self.target_network(next_states).max(1)[0].detach()
        target_q_values = rewards + (0.99 * next_q_values * ~dones)

        loss = nn.MSELoss()(current_q_values.squeeze(), target_q_values)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
```

## Mini Lab / Exercise

**Creating a Simple Object Recognition System**

In this exercise, you'll build a basic computer vision system for robot perception:

### Prerequisites:
- Python with OpenCV, numpy, and ROS2
- Camera connected to your system
- Basic knowledge of image processing

### Steps:
1. Set up a camera feed using OpenCV
2. Implement basic color-based object detection
3. Add simple shape detection
4. Create a ROS2 node that processes camera images
5. Test with different objects

### Sample Code Structure:
```python
import cv2
import numpy as np

def detect_red_objects(image):
    """Detect red objects in an image"""
    # Convert BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define range for red color
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    lower_red2 = np.array([170, 50, 50])
    upper_red2 = np.array([180, 255, 255])

    # Threshold the HSV image to get only red colors
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw bounding boxes around detected objects
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Filter small contours
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

    return image, len(contours)

# Main loop
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break

    processed_frame, count = detect_red_objects(frame)
    cv2.putText(processed_frame, f'Red objects: {count}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Object Detection', processed_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### Questions to Explore:
- How does lighting affect color-based detection?
- What other features could be used for object recognition?
- How would you extend this to recognize multiple object types?
- What challenges arise when implementing this on a moving robot?

## Next Chapter Preview

In the next chapter, "Real Hardware Integration," we'll bridge the gap between simulation and reality. You'll learn how to connect your virtual robot designs to actual physical hardware, work with real sensors and actuators, and overcome the challenges of real-world robotics.

We'll explore specific hardware platforms, sensor integration, safety considerations, and the practical aspects of deploying robots in real environments. You'll also learn about ROS2's hardware interface layer and how to create drivers for custom hardware.

This chapter will prepare you for the exciting challenge of bringing your robot designs from simulation to reality, where they can interact with the physical world.

Get ready to make your robots physical!