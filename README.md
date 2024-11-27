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

Topics
Published Topics

This topic publishes the current state of the motors.
    "/motor_status"
    
    Data Structure:
        data[0]: Motor 1 speed (in RPM)
        data[1]: Motor 1 status (e.g., error codes or operational state)
        data[2]: Motor 1 encoder value (1 full rotation = 30 counts)
        data[3]: Motor 2 speed (in RPM)
        data[4]: Motor 2 status
        data[5]: Motor 2 encoder value (1 full rotation = 30 counts)

    Notes:
        Motor rotation direction:
            Clockwise (CW): Positive values
            Counterclockwise (CCW): Negative values

Subscribed Topics

The package does not currently subscribe to any topics, but it can be extended as needed.


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
    git clone https://github.com/seongminJadenLeeunib/sendbooster_amr_server.git sendbooster_server

Build the workspace:

    cd ~/ros2_ws
    colcon build

Source the workspace:

    source ~/ros2_ws/install/setup.bash

Usage
Launching the Server

Use the provided launch file to start the sendbooster_server package:

    ros2 launch sendbooster_server sendbooster_server.launch.py


File Details

    config/sendbooster.lua: Configuration parameters for server behavior.
    launch/sendbooster_server.launch.py: Entry point for launching the package.
    msg/HeaderedTwist.msg: Custom message type for motion commands.
    sendbooster_server/sendbooster_odom.py: Computes robot odometry based on sensor data.
    sendbooster_server/sendbooster_JointStatePublisher.py: Publishes joint states to the /joint_states topic.
    urdf/amr.urdf: Describes the robot's physical structure for simulation and visualization.

1. Position Calculation

The robot's position is calculated based on the encoder values. The robot's x, y coordinates and θ (orientation angle) are updated based on the distance traveled by the two motors.
1.1 Distance Calculation

The RPM (Revolutions Per Minute) of the motors is used to calculate the number of wheel rotations, and this is then multiplied by the wheel's circumference to compute the distance traveled.
    Distance=RPM×Gear Ratio×2π×Wheel Radius×Δt
    Distance=RPM×Gear Ratio×2π×Wheel Radius×Δt

Where:

    RPM is the motor's revolutions per minute.
    Gear Ratio is the ratio between motor rotations and wheel rotations (e.g., 1:10).
    Wheel Radius is the radius of the robot's wheel.
    ΔtΔt is the time interval (in seconds).

1.2 Position Update

The robot's new position (x, y) and orientation angle θ are updated using the distances traveled by the left and right wheels, left_distance and right_distance.

    Linear Velocity: The average distance traveled by both wheels is used to calculate the linear velocity.

    Linear Velocity=left_distance+right_distance2
    Linear Velocity=2left_distance+right_distance​

    Angular Velocity: The difference in the distances traveled by the two wheels is used to calculate the angular velocity.

Angular Velocity=right_distance−left_distanceWheelbase
Angular Velocity=Wheelbaseright_distance−left_distance​

    Position Update (x, y): The robot's new position (x, y) is updated using the linear velocity and angular velocity.
    
    xnew=xold+Linear Velocity×cos⁡(θ)×Δt
    xnew​=xold​+Linear Velocity×cos(θ)×Δt
    ynew=yold+Linear Velocity×sin⁡(θ)×Δt
    ynew​=yold​+Linear Velocity×sin(θ)×Δt

    Orientation Update (θ):

    θnew=θold+Angular Velocity×Δt
    θnew​=θold​+Angular Velocity×Δt
2. Velocity Calculation

The robot's linear velocity and angular velocity are calculated from the RPM values of the two wheels.
2.1 Linear Velocity

Linear velocity is calculated as the average of the distances traveled by both wheels.

    Linear Velocity=left_distance+right_distance2
    Linear Velocity=2left_distance+right_distance​

Where left_distance and right_distance are the distances traveled by the left and right wheels, respectively.

2.2 Angular Velocity

Angular velocity is calculated based on the difference in the distances traveled by the two wheels. The higher the angular velocity, the more the robot rotates.

    Angular Velocity=right_distance−left_distanced
    Angular Velocity=dright_distance−left_distance​

Where d is the distance between the two wheels (e.g., 0.6m).

This explanation and these formulas help to understand how the robot's position and velocity are calculated based on the encoder values and wheel rotations. These calculations are used for updating the robot's odometry and controlling its movement.


License

This package is licensed under the terms specified in the LICENSE file.
Contributions

Contributions are welcome! Please fork the repository and submit a pull request.

For questions or feature requests, open an issue in the repository.
Maintainer

    Name: seongmin Jaden Lee
    Contact: roboticsmaster@naver.com / seongmin@unib.kr
