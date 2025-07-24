# Steering Lookup for ROS2

This package provides an importable library to use the steering lookup obtained from sysid experiments, ported to ROS2.

## How to import Python lib

Build the library (`colcon build --packages-select steering_lookup`) and source the workspace. Then you can import it in another ROS2 package like so:

```python
from steering_lookup.lookup_steer_angle import LookupSteerAngle

# [...]

steer_lookup = LookupSteerAngle('NUC1_pacejka')
accel = 5.0 # m/s2
vel = 3.5   # m/s
steer_angle = steer_lookup.lookup_steer_angle(accel, vel)
# Output steer angle:
# get_logger("my_node").info(f"Steering angle: {steer_angle}")
```

## Configuration

Place your lookup table CSV files in the `cfg/` directory.