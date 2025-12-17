# Environment Setup Guide

This guide outlines the recommended tools, software, and versions required to follow along with the "Physical AI Humanoid Robotics Book". It is crucial to set up your environment correctly to avoid compatibility issues.

## 1. Operating System

- **Primary Recommendation**: Ubuntu 22.04 LTS (Jammy Jellyfish)
  - Many robotics tools (especially ROS2) are best supported on Linux.
- **Alternatives**: Windows (with WSL2), macOS (support may vary for some tools).

## 2. Python

- **Version**: Python 3.9, 3.10, or 3.11
  - It's recommended to use a virtual environment (e.g., `venv` or `conda`) for project-specific dependencies.
- **Installation (Ubuntu)**:
  ```bash
  sudo apt update
  sudo apt install python3.9 python3.9-venv
  ```
- **Key Libraries (will be installed per chapter/lab)**:
  - `numpy`, `scipy` (for mathematical operations)
  - `matplotlib` (for plotting/visualization)
  - `opencv-python` (for computer vision tasks)
  - `pyserial` (for serial communication with microcontrollers)

## 3. ROS2 (Robot Operating System 2)

- **Distribution**: Humble Hawksbill (for Ubuntu 22.04)
  - Follow the official installation guide for your specific OS.
- **Installation (Ubuntu 22.04)**: Refer to the official ROS2 Humble documentation:
  - [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)
  - Key steps include setting up sources, GPG keys, installing desktop full, and sourcing the setup script.

## 4. Gazebo (Classic / Ignition Gazebo)

- **Recommendation**: Gazebo Classic (version 11) for ROS1/ROS2 Humble integration.
  - For newer projects or advanced features, Ignition Gazebo (now called Gazebo Sim).
  - The book will primarily use Gazebo Classic for simplicity with ROS2 Humble.
- **Installation (Gazebo Classic 11 on Ubuntu 22.04)**: Follow the official guide:
  - [Gazebo Classic Installation Guide](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
  - Ensure you install the `ros-humble-gazebo-ros-pkgs` for ROS2 integration.

## 5. Unity (Optional for Advanced Simulations)

- **Version**: Unity LTS Release (e.g., 2022.3 LTS or newer)
  - For creating more visually rich or complex simulation environments, especially if focusing on human-robot interaction or advanced perception.
- **Installation**: Download from the official Unity Hub:
  - [Unity Download Page](https://unity.com/download)
- **Unity Robotics Hub**: Essential package for ROS integration in Unity.
  - [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

## 6. Microcontroller IDE (for Hardware Labs)

- **Arduino IDE**: For Arduino-based projects.
  - [Arduino IDE Download](https://www.arduino.cc/en/software)
- **VS Code with PlatformIO**: A more powerful alternative for various microcontrollers.
  - [PlatformIO for VS Code](https://platformio.org/platformio-ide)

## Important Notes

- **Consistency**: Stick to the recommended versions as closely as possible to minimize issues.
- **Virtual Environments**: Always use Python virtual environments for managing project dependencies.
- **Updates**: Regularly update your system and tools, but be aware of potential breaking changes between major versions of ROS2 or Gazebo.
