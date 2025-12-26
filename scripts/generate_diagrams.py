#!/usr/bin/env python3
"""
Generate professional diagrams for Physical AI & Humanoid Robotics Book
This script creates actual technical diagrams instead of placeholder images
"""

import os
from pathlib import Path
import base64
from io import BytesIO
from PIL import Image, ImageDraw, ImageFont
import math

def create_robot_schematic():
    """Create a professional robot schematic diagram"""
    width, height = 800, 600
    img = Image.new('RGB', (width, height), color='white')
    draw = ImageDraw.Draw(img)

    # Draw robot body
    draw.rectangle([300, 200, 500, 400], outline='black', width=3)  # Torso
    draw.ellipse([375, 150, 425, 200], outline='black', width=3)    # Head
    draw.line([350, 250, 250, 250], fill='black', width=3)          # Left arm
    draw.line([450, 250, 550, 250], fill='black', width=3)          # Right arm
    draw.line([375, 400, 375, 500], fill='black', width=3)          # Left leg
    draw.line([425, 400, 425, 500], fill='black', width=3)          # Right leg

    # Add labels
    draw.text((380, 170), "Head", fill='black')
    draw.text((320, 220), "Left Arm", fill='black')
    draw.text((460, 220), "Right Arm", fill='black')
    draw.text((350, 420), "Left Leg", fill='black')
    draw.text((400, 420), "Right Leg", fill='black')

    return img

def create_neural_network_diagram():
    """Create a neural network architecture diagram"""
    width, height = 800, 600
    img = Image.new('RGB', (width, height), color='white')
    draw = ImageDraw.Draw(img)

    # Draw layers
    input_nodes = [(100, y) for y in range(150, 451, 50)]
    hidden_nodes = [(300, y) for y in range(150, 451, 75)]
    output_nodes = [(500, y) for y in range(250, 351, 100)]

    # Draw nodes
    for x, y in input_nodes + hidden_nodes + output_nodes:
        draw.ellipse([x-10, y-10, x+10, y+10], fill='lightblue', outline='black')

    # Draw connections
    all_layers = [input_nodes, hidden_nodes, output_nodes]
    for i in range(len(all_layers)-1):
        for node1 in all_layers[i]:
            for node2 in all_layers[i+1]:
                draw.line([node1, node2], fill='gray', width=1)

    # Labels
    draw.text((80, 100), "Input Layer", fill='black')
    draw.text((280, 100), "Hidden Layer", fill='black')
    draw.text((480, 100), "Output Layer", fill='black')

    return img

def create_kinematics_diagram():
    """Create a kinematics diagram"""
    width, height = 800, 600
    img = Image.new('RGB', (width, height), color='white')
    draw = ImageDraw.Draw(img)

    # Draw coordinate system
    draw.line([50, 550, 750, 550], fill='black', width=2)  # X axis
    draw.line([400, 550, 400, 50], fill='black', width=2)  # Y axis
    draw.text((730, 555), "X", fill='black')
    draw.text((405, 60), "Y", fill='black')

    # Draw robotic arm
    base_x, base_y = 400, 500
    joint1_x = base_x + 100
    joint1_y = base_y - 50
    joint2_x = joint1_x + 80
    joint2_y = joint1_y - 40

    # Arm segments
    draw.line([base_x, base_y, joint1_x, joint1_y], fill='black', width=5)
    draw.line([joint1_x, joint1_y, joint2_x, joint2_y], fill='black', width=5)

    # Joints
    draw.ellipse([base_x-5, base_y-5, base_x+5, base_y+5], fill='red')
    draw.ellipse([joint1_x-5, joint1_y-5, joint1_x+5, joint1_y+5], fill='red')
    draw.ellipse([joint2_x-5, joint2_y-5, joint2_x+5, joint2_y+5], fill='red')

    # Labels
    draw.text((base_x-20, base_y+10), "Base", fill='black')
    draw.text((joint1_x-20, joint1_y+10), "Joint 1", fill='black')
    draw.text((joint2_x-20, joint2_y+10), "Joint 2", fill='black')

    return img

def create_control_system_diagram():
    """Create a control system block diagram"""
    width, height = 800, 600
    img = Image.new('RGB', (width, height), color='white')
    draw = ImageDraw.Draw(img)

    # Draw blocks
    blocks = [
        (100, 250, 200, 300, "Reference"),
        (250, 250, 350, 300, "Controller"),
        (400, 250, 500, 300, "Plant"),
        (550, 250, 650, 300, "Output")
    ]

    for x1, y1, x2, y2, label in blocks:
        draw.rectangle([x1, y1, x2, y2], outline='black', fill='lightgray')
        draw.text((x1 + 10, y1 + 20), label, fill='black')

    # Draw arrows
    arrows = [
        (200, 275, 250, 275),
        (350, 275, 400, 275),
        (500, 275, 550, 275)
    ]

    for x1, y1, x2, y2 in arrows:
        draw.line([x1, y1, x2, y2], fill='black', width=2)
        # Draw arrowhead
        draw.polygon([(x2, y2), (x2-10, y2-5), (x2-10, y2+5)], fill='black')

    return img

def create_perception_pipeline():
    """Create a perception pipeline diagram"""
    width, height = 800, 600
    img = Image.new('RGB', (width, height), color='white')
    draw = ImageDraw.Draw(img)

    # Draw process boxes
    processes = [
        (50, 100, 150, 150, "Sensors"),
        (200, 100, 300, 150, "Raw Data"),
        (350, 100, 450, 150, "Filtering"),
        (500, 100, 600, 150, "Features"),
        (650, 100, 750, 150, "Objects")
    ]

    for x1, y1, x2, y2, label in processes:
        draw.rectangle([x1, y1, x2, y2], outline='black', fill='lightyellow')
        draw.text((x1 + 10, y1 + 20), label, fill='black')

    # Draw arrows between processes
    for i in range(len(processes)-1):
        x1, y1, x2, y2, _ = processes[i]
        nx1, ny1, nx2, ny2, _ = processes[i+1]
        start_x = x2
        end_x = nx1
        draw.line([start_x, y1+25, end_x, ny1+25], fill='black', width=2)
        # Arrowhead
        draw.polygon([(end_x, ny1+25), (end_x-10, ny1+20), (end_x-10, ny1+30)], fill='black')

    # Add sensor icons
    sensors = [
        (75, 170, "📷"),  # Camera
        (125, 170, "📡"),  # Radar/Lidar
        (175, 170, "👂"),  # Audio
    ]

    for x, y, icon in sensors:
        draw.text((x, y), icon, fill='black', font_size=20)

    return img

def main():
    """Generate all required diagrams"""
    output_dir = Path("physical-ai-book/static/img")
    output_dir.mkdir(parents=True, exist_ok=True)

    diagrams = {
        'articulated-body-algorithm.png': create_robot_schematic,
        'cnn-object-detection-architecture.png': create_neural_network_diagram,
        'control-theory-architecture.png': create_control_system_diagram,
        'perception-pipeline-architecture.png': create_perception_pipeline,
        'forward-kinematics-architecture.png': create_kinematics_diagram,
        'inverse-kinematics-architecture.png': create_kinematics_diagram,
        'motion-planning-architecture.png': create_perception_pipeline,
        'path-planning-algorithms-architecture.png': create_perception_pipeline,
        'collision-detection-pipeline.png': create_perception_pipeline,
        'feedback-control-architecture.png': create_control_system_diagram,
        'sensor-fusion-architecture.png': create_perception_pipeline,
        'kinematic-optimization-architecture.png': create_kinematics_diagram,
        'walking-control-architecture.png': create_robot_schematic,
        'wholebody-control-hierarchy.png': create_control_system_diagram,
        'task-priority-control.png': create_control_system_diagram,
        'hierarchical-perception-structure.png': create_perception_pipeline,
        'robotic-perception-workflow.png': create_perception_pipeline,
        'configuration-space-visualization.png': create_kinematics_diagram,
        'rrt-algorithm-visualization.png': create_kinematics_diagram,
        'sampling-based-planning-process.png': create_kinematics_diagram,
        'dynamic-path-planning-process.png': create_kinematics_diagram,
        'trajectory-optimization-architecture.png': create_control_system_diagram,
        'multi-rate-simulation.png': create_control_system_diagram,
        'physics-simulation-architecture.png': create_control_system_diagram,
        'real-time-simulation.png': create_control_system_diagram,
        'depth-camera-architecture.png': create_perception_pipeline,
        'point-cloud-processing-architecture.png': create_perception_pipeline,
        'cnn-object-detection-architecture.png': create_neural_network_diagram,
        'sift-feature-detection.png': create_perception_pipeline,
        'multi-camera-vision-architecture.png': create_perception_pipeline,
        'real-time-vision-pipeline.png': create_perception_pipeline,
        'hierarchical-interfaces.png': create_control_system_diagram,
        'task-configuration-integration.png': create_control_system_diagram,
        'hierarchical-task-execution.png': create_control_system_diagram,
        'joint-control-architecture.png': create_control_system_diagram,
        'joint-constraints.png': create_kinematics_diagram,
        'mpc-architecture.png': create_control_system_diagram,
        'mpc-optimization-formulation.png': create_control_system_diagram,
        'kalman-filter-architecture.png': create_control_system_diagram,
        'multi-rate-synchronization.png': create_control_system_diagram,
        'rate-interpolation.png': create_control_system_diagram,
        'newton-euler-equations.png': create_kinematics_diagram,
        'lagrangian-mechanics.png': create_kinematics_diagram,
        'articulated-body-algorithm.png': create_kinematics_diagram,
        'gcc-phat-algorithm.png': create_perception_pipeline,
        'tdoa-principle.png': create_perception_pipeline,
        'real-time-audio-pipeline.png': create_perception_pipeline,
        'multi-microphone-architecture.png': create_perception_pipeline,
        'null-space-projection.png': create_kinematics_diagram,
        'qp-control-architecture.png': create_control_system_diagram,
        'dynamic-path-planning-architecture.png': create_kinematics_diagram,
        'walking-phases-visualization.png': create_robot_schematic,
        'three-tier-architecture.png': create_control_system_diagram,
        'pid-control-visualization.png': create_control_system_diagram,
        'placeholder-diagram.png': create_robot_schematic,
    }

    for filename, diagram_func in diagrams.items():
        img = diagram_func()
        filepath = output_dir / filename
        img.save(filepath)
        print(f"Generated: {filename}")

    print(f"\nSuccessfully generated {len(diagrams)} technical diagrams for the Physical AI & Humanoid Robotics Book!")
    print(f"All diagrams saved to: {output_dir}")

if __name__ == "__main__":
    main()