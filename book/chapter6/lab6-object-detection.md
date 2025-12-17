# Chapter 6 Lab: Object Detection Using Computer Vision

## Objective

The goal of this lab is to implement a computer vision system for object detection using both traditional computer vision techniques and deep learning approaches. You will learn to process camera data, detect and recognize objects, and integrate this perception system with a robotic platform.

## Learning Outcomes

By the end of this lab, you should be able to:

- Implement traditional computer vision techniques for object detection
- Use deep learning models for object recognition
- Integrate sensor data processing with robotic systems
- Apply sensor fusion techniques for improved perception
- Evaluate perception system performance and accuracy
- Handle real-time processing constraints in perception systems

## Prerequisites

- ROS2 Humble Hawksbill installed
- Python programming skills
- Basic understanding of computer vision concepts
- OpenCV and TensorFlow/PyTorch installed
- Basic knowledge of neural networks

## Materials Needed

- Computer with ROS2 and camera access
- Python development environment
- Sample images and video datasets
- Pre-trained models (YOLO, MobileNet, etc.)

## Background

Computer vision is fundamental to robot perception, enabling robots to understand their environment through visual information. This lab will explore both traditional and modern approaches to object detection, comparing their effectiveness and applicability in robotic systems.

## Lab Procedure

### Part 1: Setting Up the Environment (15 minutes)

1. **Create a new ROS2 package for this lab:**
   ```bash
   cd ~/ros2_labs/src
   ros2 pkg create --build-type ament_python lab_object_detection --dependencies rclpy sensor_msgs cv_bridge std_msgs geometry_msgs
   ```

2. **Navigate to the package directory:**
   ```bash
   cd lab_object_detection
   ```

3. **Create the main script directory:**
   ```bash
   mkdir -p lab_object_detection
   ```

### Part 2: Installing Required Libraries (10 minutes)

1. **Install required Python packages:**
   ```bash
   pip3 install opencv-python numpy tensorflow torch torchvision
   ```

2. **Install additional ROS2 vision packages:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-vision-opencv ros-humble-cv-bridge
   ```

### Part 3: Implementing Traditional Computer Vision (45 minutes)

Create a traditional computer vision approach using OpenCV:

```python
#!/usr/bin/env python3
import cv2
import numpy as np
import math

class TraditionalObjectDetector:
    def __init__(self):
        self.min_area = 100
        self.max_area = 50000
        self.contour_threshold = 100

    def detect_by_color(self, image, lower_color, upper_color):
        """
        Detect objects based on color range
        lower_color, upper_color: HSV color range
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create mask for color range
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Apply morphological operations to clean up mask
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area < area < self.max_area:
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                detected_objects.append({
                    'type': 'color_object',
                    'center': (center_x, center_y),
                    'bbox': (x, y, w, h),
                    'area': area,
                    'confidence': 0.8  # Placeholder confidence
                })

        return detected_objects

    def detect_by_shape(self, image, min_vertices=3, max_vertices=20):
        """
        Detect objects based on shape characteristics
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area < area < self.max_area:
                # Approximate contour to polygon
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                vertices = len(approx)
                if min_vertices <= vertices <= max_vertices:
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2

                    shape_type = self.classify_shape(vertices, contour)
                    detected_objects.append({
                        'type': shape_type,
                        'center': (center_x, center_y),
                        'bbox': (x, y, w, h),
                        'area': area,
                        'vertices': vertices,
                        'confidence': 0.7  # Placeholder confidence
                    })

        return detected_objects

    def classify_shape(self, vertices, contour):
        """Classify shape based on vertices and contour properties"""
        if vertices == 3:
            return 'triangle'
        elif vertices == 4:
            # Check if it's square or rectangle
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h
            if 0.75 <= aspect_ratio <= 1.25:
                return 'square'
            else:
                return 'rectangle'
        elif vertices == 5:
            return 'pentagon'
        elif vertices == 6:
            return 'hexagon'
        else:
            # Check if it's circle-like
            area = cv2.contourArea(contour)
            if area > 0:
                perimeter = cv2.arcLength(contour, True)
                circularity = 4 * math.pi * area / (perimeter * perimeter)
                if 0.7 <= circularity <= 1.2:
                    return 'circle'
            return 'polygon'

    def detect_by_template_matching(self, image, template, threshold=0.7):
        """
        Detect objects using template matching
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

        # Perform template matching
        result = cv2.matchTemplate(gray, template_gray, cv2.TM_CCOEFF_NORMED)
        locations = np.where(result >= threshold)

        detected_objects = []
        for pt in zip(*locations[::-1]):
            x, y = pt
            h, w = template_gray.shape
            center_x = x + w // 2
            center_y = y + h // 2

            detected_objects.append({
                'type': 'template_match',
                'center': (center_x, center_y),
                'bbox': (x, y, w, h),
                'confidence': float(result[y, x])
            })

        return detected_objects
```

### Part 4: Implementing Deep Learning Object Detection (60 minutes)

Create a deep learning-based object detection system:

```python
#!/usr/bin/env python3
import cv2
import numpy as np
import tensorflow as tf
from tensorflow import keras
import torch
import torchvision
from torchvision import transforms
from PIL import Image

class DeepObjectDetector:
    def __init__(self, model_type='tensorflow'):
        self.model_type = model_type
        self.model = None
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        # Initialize model
        if model_type == 'tensorflow':
            self.load_tensorflow_model()
        else:
            self.load_pytorch_model()

    def load_tensorflow_model(self):
        """Load a pre-trained TensorFlow model (e.g., MobileNet SSD)"""
        # For this lab, we'll simulate loading a model
        # In practice, you would load a pre-trained model like SSD or YOLO
        try:
            # Load a pre-trained model (you might need to download it first)
            # self.model = tf.saved_model.load('path/to/model')
            print("TensorFlow model loaded")
        except:
            print("Using dummy model for demonstration")
            self.model = "dummy_tensorflow"

    def load_pytorch_model(self):
        """Load a pre-trained PyTorch model (e.g., YOLO or Faster R-CNN)"""
        try:
            # Load a pre-trained model
            self.model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
            self.model.eval()
            print("PyTorch model loaded")
        except:
            print("Using dummy model for demonstration")
            self.model = "dummy_pytorch"

    def preprocess_image(self, image):
        """Preprocess image for deep learning model"""
        # Convert BGR to RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Convert to PIL Image
        pil_image = Image.fromarray(image_rgb)

        # Define transforms
        transform = transforms.Compose([
            transforms.Resize((416, 416)),  # Resize to model input size
            transforms.ToTensor(),
        ])

        # Apply transforms
        tensor_image = transform(pil_image)
        tensor_image = tensor_image.unsqueeze(0)  # Add batch dimension

        return tensor_image

    def detect_objects_tensorflow(self, image):
        """Detect objects using TensorFlow model"""
        # This is a simplified implementation
        # In practice, you would run inference on the loaded model

        # For demonstration, return mock detections
        height, width = image.shape[:2]

        # Simulate detection results
        detections = []

        # Add some mock detections
        if np.random.random() > 0.3:  # 70% chance of detecting something
            x = int(np.random.uniform(0.1, 0.9) * width)
            y = int(np.random.uniform(0.1, 0.9) * height)
            w = int(np.random.uniform(0.1, 0.3) * width)
            h = int(np.random.uniform(0.1, 0.3) * height)

            detections.append({
                'type': self.class_names[np.random.randint(0, len(self.class_names))],
                'center': (x + w//2, y + h//2),
                'bbox': (x, y, w, h),
                'confidence': np.random.uniform(0.5, 0.99)
            })

        return detections

    def detect_objects_pytorch(self, image):
        """Detect objects using PyTorch model"""
        # Preprocess image
        input_tensor = self.preprocess_image(image)

        # Run inference
        with torch.no_grad():
            if isinstance(self.model, str) and "dummy" in self.model:
                # Mock detection for demonstration
                height, width = image.shape[:2]
                detections = []

                if np.random.random() > 0.4:  # 60% chance of detecting something
                    x = int(np.random.uniform(0.1, 0.9) * width)
                    y = int(np.random.uniform(0.1, 0.9) * height)
                    w = int(np.random.uniform(0.1, 0.3) * width)
                    h = int(np.random.uniform(0.1, 0.3) * height)

                    detections.append({
                        'type': self.class_names[np.random.randint(0, len(self.class_names))],
                        'center': (x + w//2, y + h//2),
                        'bbox': (x, y, w, h),
                        'confidence': np.random.uniform(0.5, 0.99)
                    })

                return detections
            else:
                # Actual model inference
                outputs = self.model(input_tensor)

                # Process outputs
                detections = []
                for i in range(len(outputs[0]['boxes'])):
                    box = outputs[0]['boxes'][i].numpy()
                    score = outputs[0]['scores'][i].item()
                    label = outputs[0]['labels'][i].item()

                    if score > 0.5:  # Confidence threshold
                        x1, y1, x2, y2 = box
                        x, y, w, h = int(x1), int(y1), int(x2-x1), int(y2-y1)

                        detections.append({
                            'type': self.class_names[label-1] if label-1 < len(self.class_names) else f'object_{label}',
                            'center': (x + w//2, y + h//2),
                            'bbox': (x, y, w, h),
                            'confidence': score
                        })

                return detections

    def detect_objects(self, image):
        """Main detection function"""
        if self.model_type == 'tensorflow':
            return self.detect_objects_tensorflow(image)
        else:
            return self.detect_objects_pytorch(image)
```

### Part 5: Creating the ROS2 Perception Node (40 minutes)

Create the main ROS2 node that integrates both traditional and deep learning approaches:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from lab_object_detection.traditional_detector import TraditionalObjectDetector
from lab_object_detection.deep_detector import DeepObjectDetector

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize detectors
        self.traditional_detector = TraditionalObjectDetector()
        self.deep_detector = DeepObjectDetector(model_type='pytorch')  # Use PyTorch for this example

        # Publishers
        self.detection_pub = self.create_publisher(String, 'object_detections', 10)
        self.visualization_pub = self.create_publisher(Image, 'detection_visualization', 10)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)

        # Parameters
        self.use_traditional = self.declare_parameter('use_traditional', True).value
        self.use_deep_learning = self.declare_parameter('use_deep_learning', True).value
        self.confidence_threshold = self.declare_parameter('confidence_threshold', 0.5).value

        self.get_logger().info('Perception node initialized')

    def image_callback(self, msg):
        """Process incoming image and detect objects"""
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Detect objects using traditional methods
        traditional_detections = []
        if self.use_traditional:
            # Detect red objects as example
            red_lower = np.array([0, 50, 50])
            red_upper = np.array([10, 255, 255])
            traditional_detections = self.traditional_detector.detect_by_color(
                cv_image, red_lower, red_upper)

        # Detect objects using deep learning
        deep_detections = []
        if self.use_deep_learning:
            deep_detections = self.deep_detector.detect_objects(cv_image)

        # Combine and filter detections
        all_detections = traditional_detections + deep_detections
        filtered_detections = [
            det for det in all_detections
            if det['confidence'] >= self.confidence_threshold
        ]

        # Create visualization
        vis_image = self.draw_detections(cv_image, filtered_detections)

        # Publish detections
        self.publish_detections(filtered_detections)

        # Publish visualization
        vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding="bgr8")
        vis_msg.header = msg.header
        self.visualization_pub.publish(vis_msg)

        self.get_logger().info(f'Detected {len(filtered_detections)} objects')

    def draw_detections(self, image, detections):
        """Draw detection results on image"""
        vis_image = image.copy()

        for detection in detections:
            x, y, w, h = detection['bbox']
            center_x, center_y = detection['center']

            # Draw bounding box
            cv2.rectangle(vis_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Draw center point
            cv2.circle(vis_image, (center_x, center_y), 5, (255, 0, 0), -1)

            # Draw label and confidence
            label = f"{detection['type']}: {detection['confidence']:.2f}"
            cv2.putText(vis_image, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return vis_image

    def publish_detections(self, detections):
        """Publish detection results"""
        if not detections:
            return

        # Create detection message (simplified)
        detection_str = ""
        for i, detection in enumerate(detections):
            if i > 0:
                detection_str += "; "
            detection_str += f"{detection['type']} at ({detection['center'][0]}, {detection['center'][1]}) conf={detection['confidence']:.2f}"

        detection_msg = String()
        detection_msg.data = detection_str
        self.detection_pub.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()

    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Part 6: Implementing Sensor Fusion (30 minutes)

Create a sensor fusion module that combines multiple sensor inputs:

```python
#!/usr/bin/env python3
import numpy as np
from scipy.spatial.distance import cdist
import cv2

class SensorFusion:
    def __init__(self):
        self.confidence_weights = {
            'camera': 0.7,
            'lidar': 0.8,
            'radar': 0.5
        }
        self.spatial_threshold = 50  # pixels for camera, adjust as needed

    def fuse_detections(self, camera_detections, lidar_detections, radar_detections=None):
        """
        Fuse detections from multiple sensors
        Each detection should have: type, center, bbox, confidence
        """
        # Start with camera detections as primary
        fused_detections = []

        for cam_det in camera_detections:
            # Find corresponding detections from other sensors
            matched_lidar = self.find_matches(cam_det, lidar_detections)
            matched_radar = self.find_matches(cam_det, radar_detections) if radar_detections else []

            # Calculate fused confidence
            fused_confidence = self.calculate_fused_confidence(
                cam_det, matched_lidar, matched_radar
            )

            # Update detection with fused information
            fused_detection = {
                'type': cam_det['type'],
                'center': cam_det['center'],
                'bbox': cam_det['bbox'],
                'confidence': fused_confidence,
                'source_sensors': ['camera'] + ['lidar'] * len(matched_lidar) + ['radar'] * len(matched_radar)
            }

            fused_detections.append(fused_detection)

        # Add unmatched lidar/radar detections
        fused_detections.extend(self.add_unmatched_detections(
            camera_detections, lidar_detections, 'lidar'
        ))
        if radar_detections:
            fused_detections.extend(self.add_unmatched_detections(
                camera_detections + lidar_detections, radar_detections, 'radar'
            ))

        # Filter by minimum confidence
        fused_detections = [det for det in fused_detections if det['confidence'] > 0.3]

        return fused_detections

    def find_matches(self, primary_detection, secondary_detections):
        """Find matching detections in secondary sensor data"""
        matches = []
        primary_center = np.array(primary_detection['center']).reshape(1, -1)

        for sec_det in secondary_detections:
            sec_center = np.array(sec_det['center']).reshape(1, -1)
            distance = np.linalg.norm(primary_center - sec_center)

            if distance < self.spatial_threshold:
                matches.append(sec_det)

        return matches

    def calculate_fused_confidence(self, cam_det, lidar_matches, radar_matches):
        """Calculate fused confidence based on matched detections"""
        # Weighted average of confidences
        total_weight = self.confidence_weights['camera']
        weighted_sum = cam_det['confidence'] * self.confidence_weights['camera']

        for lidar_det in lidar_matches:
            total_weight += self.confidence_weights['lidar']
            weighted_sum += lidar_det['confidence'] * self.confidence_weights['lidar']

        for radar_det in radar_matches:
            total_weight += self.confidence_weights['radar']
            weighted_sum += radar_det['confidence'] * self.confidence_weights['radar']

        if total_weight > 0:
            fused_confidence = weighted_sum / total_weight
        else:
            fused_confidence = cam_det['confidence']

        # Ensure confidence doesn't exceed 1.0
        return min(fused_confidence, 1.0)

    def add_unmatched_detections(self, primary_detections, secondary_detections, sensor_type):
        """Add unmatched secondary detections to fused results"""
        unmatched = []

        for sec_det in secondary_detections:
            is_matched = False
            for prim_det in primary_detections:
                distance = np.linalg.norm(
                    np.array(sec_det['center']) - np.array(prim_det['center'])
                )
                if distance < self.spatial_threshold:
                    is_matched = True
                    break

            if not is_matched:
                # Boost confidence slightly for unmatched detections if they're highly confident
                if sec_det['confidence'] > 0.7:
                    unmatched.append({
                        'type': sec_det['type'],
                        'center': sec_det['center'],
                        'bbox': sec_det['bbox'],
                        'confidence': sec_det['confidence'] * 0.8,  # Slightly reduce confidence for unmatched
                        'source_sensors': [sensor_type]
                    })

        return unmatched

    def track_objects_over_time(self, current_detections, previous_detections, max_distance=100):
        """Simple object tracking by matching detections across frames"""
        if not previous_detections:
            # Assign IDs to first frame detections
            for i, det in enumerate(current_detections):
                det['track_id'] = i
            return current_detections

        # Create distance matrix
        current_centers = np.array([det['center'] for det in current_detections])
        previous_centers = np.array([det['center'] for det in previous_detections])

        # Calculate distances
        distances = cdist(current_centers, previous_centers)

        # Simple assignment (nearest neighbor)
        assigned_prev = set()
        assigned_curr = set()

        for i in range(len(current_detections)):
            if i in assigned_curr:
                continue

            # Find closest previous detection
            min_dist_idx = np.argmin(distances[i])
            min_dist = distances[i][min_dist_idx]

            if min_dist < max_distance and min_dist_idx not in assigned_prev:
                # Assign same track ID
                current_detections[i]['track_id'] = previous_detections[min_dist_idx].get('track_id', min_dist_idx)
                assigned_prev.add(min_dist_idx)
                assigned_curr.add(i)

        # Assign new IDs to unassigned detections
        next_id = max([det.get('track_id', -1) for det in previous_detections], default=-1) + 1
        for i in range(len(current_detections)):
            if i not in assigned_curr:
                current_detections[i]['track_id'] = next_id
                next_id += 1

        return current_detections
```

### Part 7: Creating a Performance Evaluation Module (25 minutes)

Create a module to evaluate the performance of the detection system:

```python
#!/usr/bin/env python3
import numpy as np
from sklearn.metrics import precision_score, recall_score, f1_score

class DetectionEvaluator:
    def __init__(self):
        self.detection_history = []

    def calculate_iou(self, box1, box2):
        """Calculate Intersection over Union between two bounding boxes"""
        x1_1, y1_1, w1, h1 = box1
        x2_1, y2_1, w2, h2 = box2

        x1_2, y1_2 = x1_1 + w1, y1_1 + h1
        x2_2, y2_2 = x2_1 + w2, y2_1 + h2

        # Calculate intersection
        x_left = max(x1_1, x2_1)
        y_top = max(y1_1, y2_1)
        x_right = min(x1_2, x2_2)
        y_bottom = min(y1_2, y2_2)

        if x_right < x_left or y_bottom < y_top:
            return 0.0

        intersection_area = (x_right - x_left) * (y_bottom - y_top)
        box1_area = w1 * h1
        box2_area = w2 * h2
        union_area = box1_area + box2_area - intersection_area

        return intersection_area / union_area if union_area > 0 else 0.0

    def evaluate_detection(self, predicted_detections, ground_truth_detections, iou_threshold=0.5):
        """
        Evaluate detection performance
        predicted_detections: list of detection dicts
        ground_truth_detections: list of detection dicts with ground truth
        """
        if not ground_truth_detections and not predicted_detections:
            return {'precision': 1.0, 'recall': 1.0, 'f1': 1.0, 'accuracy': 1.0}

        if not ground_truth_detections:
            return {'precision': 0.0, 'recall': 0.0, 'f1': 0.0, 'accuracy': 0.0}

        if not predicted_detections:
            return {'precision': 0.0, 'recall': 0.0, 'f1': 0.0, 'accuracy': 0.0}

        # Create matching matrix
        matches = []
        for i, pred_det in enumerate(predicted_detections):
            for j, gt_det in enumerate(ground_truth_detections):
                iou = self.calculate_iou(pred_det['bbox'], gt_det['bbox'])
                if iou >= iou_threshold:
                    matches.append((i, j, iou))

        # Count true positives, false positives, false negatives
        matched_predictions = set()
        matched_ground_truth = set()

        # Sort matches by IoU in descending order and apply greedy matching
        matches.sort(key=lambda x: x[2], reverse=True)

        for pred_idx, gt_idx, iou in matches:
            if pred_idx not in matched_predictions and gt_idx not in matched_ground_truth:
                matched_predictions.add(pred_idx)
                matched_ground_truth.add(gt_idx)

        tp = len(matched_predictions)  # True positives
        fp = len(predicted_detections) - tp  # False positives
        fn = len(ground_truth_detections) - len(matched_ground_truth)  # False negatives

        precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0

        # Calculate mean Average Precision (mAP) if needed
        # For simplicity, we'll return basic metrics
        accuracy = tp / (tp + fp + fn) if (tp + fp + fn) > 0 else 0.0

        return {
            'precision': precision,
            'recall': recall,
            'f1': f1,
            'accuracy': accuracy,
            'true_positives': tp,
            'false_positives': fp,
            'false_negatives': fn,
            'total_predictions': len(predicted_detections),
            'total_ground_truth': len(ground_truth_detections)
        }

    def evaluate_real_time_performance(self, processing_times):
        """Evaluate real-time performance metrics"""
        if not processing_times:
            return {}

        return {
            'mean_processing_time': np.mean(processing_times),
            'std_processing_time': np.std(processing_times),
            'min_processing_time': np.min(processing_times),
            'max_processing_time': np.max(processing_times),
            'fps': 1.0 / np.mean(processing_times) if np.mean(processing_times) > 0 else 0,
            'latency_percentiles': {
                '50th': np.percentile(processing_times, 50),
                '90th': np.percentile(processing_times, 90),
                '95th': np.percentile(processing_times, 95),
                '99th': np.percentile(processing_times, 99)
            }
        }

    def log_detection_performance(self, frame_id, metrics, processing_time):
        """Log performance metrics for analysis"""
        log_entry = {
            'frame_id': frame_id,
            'timestamp': self.get_current_time(),
            'metrics': metrics,
            'processing_time': processing_time
        }
        self.detection_history.append(log_entry)

    def get_current_time(self):
        """Get current timestamp"""
        import time
        return time.time()
```

### Part 8: Creating a Test and Evaluation Script (20 minutes)

Create a script to test and evaluate the perception system:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from lab_object_detection.evaluator import DetectionEvaluator

class PerceptionTester(Node):
    def __init__(self):
        super().__init__('perception_tester')

        self.bridge = CvBridge()
        self.evaluator = DetectionEvaluator()

        # For this test, we'll generate synthetic images
        self.test_counter = 0
        self.max_tests = 100

        # Timer for synthetic image generation
        self.timer = self.create_timer(0.1, self.generate_test_image)

        self.get_logger().info('Perception tester initialized')

    def generate_test_image(self):
        """Generate synthetic test images with known objects"""
        if self.test_counter >= self.max_tests:
            self.get_logger().info('Completed all test images')
            self.timer.cancel()
            return

        # Create a synthetic image with known objects
        height, width = 480, 640
        image = np.zeros((height, width, 3), dtype=np.uint8)

        # Add some synthetic objects
        # Red rectangle (simulating a stop sign)
        cv2.rectangle(image, (100, 100), (200, 200), (0, 0, 255), -1)
        cv2.rectangle(image, (100, 100), (200, 200), (0, 0, 0), 2)

        # Blue circle (simulating a ball)
        cv2.circle(image, (400, 150), 50, (255, 0, 0), -1)
        cv2.circle(image, (400, 150), 50, (0, 0, 0), 2)

        # Green triangle (simulating a yield sign)
        triangle_points = np.array([[300, 300], [350, 250], [400, 300]], np.int32)
        cv2.fillPoly(image, [triangle_points], (0, 255, 0))
        cv2.polylines(image, [triangle_points], True, (0, 0, 0), 2)

        # Convert to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = f"test_frame_{self.test_counter}"

        # For evaluation, we know the ground truth
        ground_truth = [
            {
                'type': 'stop_sign',
                'center': (150, 150),
                'bbox': (100, 100, 100, 100),
                'confidence': 1.0
            },
            {
                'type': 'ball',
                'center': (400, 150),
                'bbox': (350, 100, 100, 100),  # Approximate
                'confidence': 1.0
            },
            {
                'type': 'yield_sign',
                'center': (350, 283),  # Approximate center of triangle
                'bbox': (300, 250, 100, 50),  # Approximate
                'confidence': 1.0
            }
        ]

        # Process the image through our perception pipeline
        # (In a real scenario, this would be done by the perception node)
        self.process_image_for_evaluation(image, ground_truth, self.test_counter)

        self.test_counter += 1

    def process_image_for_evaluation(self, image, ground_truth, frame_id):
        """Process image and evaluate performance"""
        import time
        from lab_object_detection.traditional_detector import TraditionalObjectDetector
        from lab_object_detection.deep_detector import DeepObjectDetector

        start_time = time.time()

        # Use traditional detector for this evaluation
        detector = TraditionalObjectDetector()

        # Detect red objects (stop sign)
        red_lower = np.array([0, 50, 50])
        red_upper = np.array([10, 255, 255])
        red_detections = detector.detect_by_color(image, red_lower, red_upper)

        # Detect blue objects (ball)
        blue_lower = np.array([100, 50, 50])
        blue_upper = np.array([130, 255, 255])
        blue_detections = detector.detect_by_color(image, blue_lower, blue_upper)

        # Detect green objects (yield sign)
        green_lower = np.array([40, 50, 50])
        green_upper = np.array([80, 255, 255])
        green_detections = detector.detect_by_color(image, green_lower, green_upper)

        # Combine all detections
        predicted_detections = red_detections + blue_detections + green_detections

        processing_time = time.time() - start_time

        # Evaluate performance
        metrics = self.evaluator.evaluate_detection(predicted_detections, ground_truth)

        # Log results
        self.evaluator.log_detection_performance(frame_id, metrics, processing_time)

        self.get_logger().info(
            f'Test {frame_id}: Precision={metrics["precision"]:.3f}, '
            f'Recall={metrics["recall"]:.3f}, F1={metrics["f1"]:.3f}, '
            f'Time={processing_time:.4f}s'
        )

def main(args=None):
    rclpy.init(args=args)
    tester = PerceptionTester()

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

### Part 9: Creating a Launch File (10 minutes)

Create a launch file to run the complete perception system:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab_object_detection',
            executable='perception_node',
            name='perception_node',
            parameters=[
                {'use_traditional': True},
                {'use_deep_learning': True},
                {'confidence_threshold': 0.5}
            ],
            output='screen'
        ),
        Node(
            package='lab_object_detection',
            executable='perception_tester',
            name='perception_tester',
            output='screen'
        )
    ])
```

### Part 10: Updating setup.py and Building (15 minutes)

Update the setup.py file:

```python
from setuptools import setup

package_name = 'lab_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Object detection lab for robotics perception',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = lab_object_detection.perception_node:main',
            'perception_tester = lab_object_detection.perception_tester:main',
        ],
    },
)
```

Build the package:

```bash
cd ~/ros2_labs
colcon build --packages-select lab_object_detection
source install/setup.bash
```

## Discussion Questions

1. How do traditional computer vision methods compare to deep learning approaches in terms of accuracy, speed, and robustness?

2. What are the advantages and disadvantages of sensor fusion in robotic perception?

3. How does real-time processing constrain the choice of perception algorithms?

4. What evaluation metrics are most important for robotic perception systems?

5. How would you handle occlusion and partial visibility in object detection?

## Extension Activities

1. **Advanced Deep Learning**: Implement YOLO or other state-of-the-art detection models
2. **3D Object Detection**: Extend to use depth information from RGB-D cameras
3. **Multi-camera Fusion**: Combine data from multiple cameras for better coverage
4. **Tracking**: Implement object tracking across video frames
5. **Performance Optimization**: Optimize for edge computing with TensorRT or OpenVINO

## Assessment

Complete the following self-assessment:

- I can implement traditional computer vision object detection: [ ] Yes [ ] No [ ] Partially
- I understand deep learning approaches to object detection: [ ] Yes [ ] No [ ] Partially
- I can integrate perception with ROS2: [ ] Yes [ ] No [ ] Partially
- I can evaluate perception system performance: [ ] Yes [ ] No [ ] Partially
- I understand sensor fusion concepts: [ ] Yes [ ] No [ ] Partially

## Troubleshooting Tips

- **Performance Issues**: Use smaller input sizes or simpler models for real-time applications
- **False Detections**: Adjust confidence thresholds and implement non-maximum suppression
- **Color Detection**: Calibrate for lighting conditions and use appropriate color spaces
- **Memory Issues**: Process images in smaller batches or use memory-efficient models
- **Timing Issues**: Profile code to identify bottlenecks and optimize critical paths

## Conclusion

This lab has provided hands-on experience with computer vision for robotics, covering both traditional and deep learning approaches to object detection. You've learned to implement perception systems, integrate them with ROS2, and evaluate their performance for robotic applications.

## Resources for Further Exploration

- OpenCV documentation and tutorials
- TensorFlow and PyTorch object detection guides
- ROS2 vision packages documentation
- Research papers on object detection and perception
- Computer vision and deep learning courses
- Benchmark datasets like COCO, PASCAL VOC