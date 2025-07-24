# MAP Controller for ROS2

This package provides a Model- and Acceleration-based Pursuit (MAP) controller for autonomous racing in ROS2.

## Overview

The MAP controller is a novel trajectory tracking algorithm for high-speed autonomous racing. It calculates acceleration via L1 guidance and converts this value to a steering angle using a lookup table generated from system model parameters and tire dynamics.

This package includes:
- MAP Controller: The main controller implementation
- Pure Pursuit Controller: A traditional geometric controller for comparison
- Time Error Tracker: For tracking performance metrics

## Usage

To run the MAP controller:

```bash
ros2 launch map_controller sim_MAP.launch.py