# README for sendbooster_server Package
## Overview

sendbooster_server is a ROS 2 package designed to manage robot state publishing, odometry calculation, and configuration for a robotic system. It includes custom message types, URDF descriptions, and launch files to facilitate integration with ROS 2-based systems.
Package Structure

The directory structure of the sendbooster_server package is as follows:

.
├── config
│   └── sendbooster.lua                 # Configuration file for Lua-based setup
├── launch
│   └── sendbooster_server.launch.py    # Main launch file for the package
├── LICENSE                             # License file
├── msg
│   └── HeaderedTwist.msg               # Custom message definition
├── package.xml                         # Package metadata and dependencies
├── resource
│   └── sendbooster_server              # Resource marker for package identification
├── sendbooster_server
│   ├── __init__.py                     # Init file for Python module
│   ├── sendbooster_JointStatePublisher.py  # Publishes joint states of the robot
│   └── sendbooster_odom.py             # Calculates and publishes odometry
├── setup.cfg                           # Configuration for Python packaging
├── setup.py                            # Build and install script
├── test
│   ├── test_copyright.py               # Copyright compliance tests
│   ├── test_flake8.py                  # Linting tests using flake8
│   └── test_pep257.py                  # Docstring compliance tests
└── urdf
    └── amr.urdf                        # URDF file describing the robot model

Features

    Odometry Publishing: Calculates and publishes odometry data using the sendbooster_odom.py script.
    Joint State Publishing: Publishes the joint states of the robot using the sendbooster_JointStatePublisher.py script.
    Custom Message Types: Defines the HeaderedTwist.msg for advanced message handling.
    URDF Integration: Provides a detailed URDF file (amr.urdf) for robot model visualization and simulation.
    Testing: Includes Python scripts for ensuring code quality and compliance.
    Launch System: Easily start the server with the provided launch file.

Installation

Clone the repository into your ROS 2 workspace:

    cd ~/ros2_ws/src
    git clone <repository_url> sendbooster_server

Build the workspace:

    cd ~/ros2_ws
    colcon build

Source the workspace:

    source ~/ros2_ws/install/setup.bash

Usage
Launching the Server

Use the provided launch file to start the sendbooster_server package:

ros2 launch sendbooster_server sendbooster_server.launch.py

Custom Message

The HeaderedTwist.msg defines the following structure:

std_msgs/Header header
geometry_msgs/Twist twist

This allows for transmitting twist messages with additional metadata.
Testing

Run the included tests to ensure code quality:

colcon test
colcon test-result

File Details

    config/sendbooster.lua: Configuration parameters for server behavior.
    launch/sendbooster_server.launch.py: Entry point for launching the package.
    msg/HeaderedTwist.msg: Custom message type for motion commands.
    sendbooster_server/sendbooster_odom.py: Computes robot odometry based on sensor data.
    sendbooster_server/sendbooster_JointStatePublisher.py: Publishes joint states to the /joint_states topic.
    urdf/amr.urdf: Describes the robot's physical structure for simulation and visualization.

License

This package is licensed under the terms specified in the LICENSE file.
Contributions

Contributions are welcome! Please fork the repository and submit a pull request.

For questions or feature requests, open an issue in the repository.
Maintainer

    Name: [Your Name or Organization]
    Contact: [Your Email or Website]
