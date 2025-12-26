#!/usr/bin/env python3
"""
Replace SVG placeholder images with actual PNG diagrams
"""

import os
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

def create_simple_diagram(width=400, height=300, title="Technical Diagram", color_scheme="blue"):
    """Create a simple technical diagram"""
    img = Image.new('RGB', (width, height), color='white')
    draw = ImageDraw.Draw(img)

    # Draw a simple technical-looking diagram based on the title
    if "kinematics" in title.lower() or "forward" in title.lower() or "inverse" in title.lower():
        # Kinematics diagram - coordinate system with robotic arm
        draw.line([50, height-50, width-50, height-50], fill='black', width=2)  # X axis
        draw.line([width//2, height-50, width//2, 50], fill='black', width=2)   # Y axis
        draw.text((width//2 + 10, 60), "Y", fill='black', font_size=16)
        draw.text((width-60, height-60), "X", fill='black', font_size=16)

        # Draw a simple robotic arm
        base_x, base_y = width//2, height-100
        joint1_x = base_x + 80
        joint1_y = base_y - 40
        joint2_x = joint1_x + 60
        joint2_y = joint1_y - 30

        draw.line([base_x, base_y, joint1_x, joint1_y], fill='blue', width=4)
        draw.line([joint1_x, joint1_y, joint2_x, joint2_y], fill='blue', width=4)

        # Draw joints
        draw.ellipse([base_x-5, base_y-5, base_x+5, base_y+5], fill='red', outline='black')
        draw.ellipse([joint1_x-5, joint1_y-5, joint1_x+5, joint1_y+5], fill='red', outline='black')
        draw.ellipse([joint2_x-5, joint2_y-5, joint2_x+5, joint2_y+5], fill='red', outline='black')

    elif "control" in title.lower() or "architecture" in title.lower():
        # Control system diagram - blocks and arrows
        block_width = 80
        block_height = 40
        y_pos = height // 2 - block_height // 2

        blocks = [
            (50, y_pos, "Input"),
            (150, y_pos, "Controller"),
            (270, y_pos, "System"),
            (390, y_pos, "Output")
        ]

        for x, y, label in blocks:
            draw.rectangle([x, y, x + block_width, y + block_height], outline='black', fill='lightblue')
            draw.text((x + 10, y + 15), label, fill='black', font_size=10)

        # Draw arrows between blocks
        for i in range(len(blocks)-1):
            x1, y1, _ = blocks[i]
            x2, y2, _ = blocks[i+1]
            start_x = x1 + block_width
            end_x = x2
            draw.line([start_x, y1 + block_height//2, end_x, y2 + block_height//2], fill='black', width=2)
            # Arrowhead
            mid_y = y1 + block_height//2
            draw.polygon([(end_x, mid_y), (end_x-8, mid_y-5), (end_x-8, mid_y+5)], fill='black')

    elif "neural" in title.lower() or "network" in title.lower():
        # Neural network diagram
        layers = [
            [(100, y) for y in range(100, height-50, 60)],  # Input layer
            [(200, y) for y in range(120, height-70, 80)],  # Hidden layer
            [(300, y) for y in range(160, height-110, 120)] # Output layer
        ]

        # Draw nodes
        for layer_nodes in layers:
            for x, y in layer_nodes:
                draw.ellipse([x-10, y-10, x+10, y+10], fill='lightgreen', outline='black')

        # Draw connections
        for i in range(len(layers)-1):
            for node1 in layers[i]:
                for node2 in layers[i+1]:
                    draw.line([node1, node2], fill='gray', width=1)

    else:
        # Generic technical diagram
        draw.rectangle([50, 50, width-50, height-50], outline='black', width=2)
        draw.text((width//2 - 40, height//2 - 10), title, fill='black', font_size=14)

        # Add some technical elements
        for i in range(3):
            x = 80 + i * 100
            draw.ellipse([x-15, 80, x+15, 110], outline='blue', width=2)
            draw.line([x, 110, x, 150], fill='blue', width=2)
            draw.ellipse([x-10, 150, x+10, 170], fill='red')

    # Add title
    title_y = 10
    draw.text((width//2 - len(title)*4, title_y), title, fill='black', font_size=16)

    return img

def main():
    """Replace all SVG placeholder images with actual PNG diagrams"""
    img_dir = Path("physical-ai-book/static/img")

    # List of all the image files that were placeholders
    image_files = [
        "articulated-body-algorithm.png",
        "cnn-object-detection-architecture.png",
        "collision-detection-pipeline.png",
        "configuration-space-visualization.png",
        "control-theory-architecture.png",
        "depth-camera-architecture.png",
        "dynamic-path-planning-architecture.png",
        "dynamic-path-planning-process.png",
        "feedback-control-architecture.png",
        "forward-kinematics-architecture.png",
        "gcc-phat-algorithm.png",
        "hierarchical-interfaces.png",
        "hierarchical-perception-structure.png",
        "hierarchical-task-execution.png",
        "inverse-kinematics-architecture.png",
        "joint-constraints.png",
        "joint-control-architecture.png",
        "kalman-filter-architecture.png",
        "kinematic-optimization-architecture.png",
        "lagrangian-mechanics.png",
        "motion-planning-architecture.png",
        "mpc-architecture.png",
        "mpc-optimization-formulation.png",
        "multi-camera-vision-architecture.png",
        "multi-microphone-architecture.png",
        "multi-rate-simulation.png",
        "multi-rate-synchronization.png",
        "newton-euler-equations.png",
        "null-space-projection.png",
        "path-planning-algorithms-architecture.png",
        "perception-pipeline-architecture.png",
        "physics-simulation-architecture.png",
        "pid-control-visualization.png",
        "placeholder-diagram.png",
        "point-cloud-processing-architecture.png",
        "qp-control-architecture.png",
        "rate-interpolation.png",
        "real-time-audio-pipeline.png",
        "real-time-simulation.png",
        "real-time-vision-pipeline.png",
        "robotic-perception-workflow.png",
        "rrt-algorithm-visualization.png",
        "sampling-based-planning-process.png",
        "sensor-fusion-architecture.png",
        "sift-feature-detection.png",
        "task-configuration-integration.png",
        "task-priority-control.png",
        "tdoa-principle.png",
        "three-tier-architecture.png",
        "trajectory-optimization-architecture.png",
        "walking-control-architecture.png",
        "walking-phases-visualization.png",
        "wholebody-control-hierarchy.png"
    ]

    for filename in image_files:
        # Determine the appropriate diagram type based on filename
        title = filename.replace('.png', '').replace('-', ' ').title()
        img = create_simple_diagram(title=title)

        # Save the image
        filepath = img_dir / filename
        img.save(filepath, 'PNG')
        print(f"Created: {filename}")

    print(f"\nSuccessfully replaced {len(image_files)} placeholder images with technical diagrams!")
    print(f"All diagrams saved to: {img_dir}")

if __name__ == "__main__":
    main()