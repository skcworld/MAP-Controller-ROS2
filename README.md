# Model- and Acceleration-based Pursuit (MAP) controller for ROS2

Welcome to **MAP-Controller-ROS2**, a ROS 2 port of the original [Model- and Acceleration-based Pursuit (MAP) controller](https://github.com/ETH-PBL/MAP-Controller) developed by the [PBL Lab at ETH Zürich](https://pbl.ee.ethz.ch/). This controller provides an accurate and robust trajectory tracking algorithm for high-speed autonomous vehicles based on a combination of Pure Pursuit geometry and physics-based modeling.

### ⚠️ **This repository contains only the ROS 2 port of the MAP controller.** For a detailed explanation of the algorithm and theory, please refer to the original repository linked above.

---

## About MAP Controller

The MAP controller builds upon the principles of Pure Pursuit but enhances it with model-based acceleration computation and a lookup-table-based steering angle conversion. It is designed for high-performance driving scenarios, including autonomous racing.

The original ROS 1 implementation provided:
- A controller for real-world deployment
- A simulator for evaluation
- A lookup table generator based on vehicle dynamics

---

## What’s in This Port (ROS 2)

This repository includes:

- ROS 2 nodes implementing the MAP and Pure Pursuit controllers
- Lookup table loader for converting lateral acceleration to steering angle
- Launch files for running the controller stack
- Clean separation of logic and ROS interfaces

---

## ⚠️ Key Differences from Original Code

- **Lookup Table Generator Not Included**  
  The ROS 2 version does **not** include `simulate_model.py` or any lookup table generation logic.  
  To use the controller:
  1. Go to the [original repository](https://github.com/ETH-PBL/MAP-Controller)
  2. Run `simulate_model.py` to generate a `.csv` lookup table
  3. Place the generated file into:
     ```
     MAP-Controller-ROS2/steering_lookup/cfg/
     ```

- **ROS 1 Simulator Not Ported**  
  The `F110_ROS_Simulator` package from the original code was not ported, as this repository is focused on real vehicle deployment.

  If you need a simulator, we recommend using or adapting:
  👉 [f1tenth/f1tenth_gym_ros (ROS 2)](https://github.com/f1tenth/f1tenth_gym_ros)

  I will provide a simulator port in the future!!

---

## Build and Run Instructions
Assuming you already have a ROS 2 workspace set up:

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
```bash
# Clone the repository
git clone https://github.com/your-username/MAP-Controller_ROS2.git
```
```bash
# Navigate to workspace
cd ~/ros2_ws
```
```bash
# Install ROS2 dependencies
rosdep install --from-paths src --ignore-src -r -y
```
```bash
# Build packages
colcon build --symlink-install
```
```bash
# Setup environment
source install/setup.bash
```
```bash
# Running the Controller
ros2 launch map_controller sim_MAP.launch.py
ros2 launch map_controller sim_PP.launch.py
```

### To use a new Steering Lookup table:
1. Run simulate_model.py from the original repository to generate a table
2. Copy the generated CSV file to the steering_lookup/cfg/ folder
3. Set the LU_table parameter in map_controller to the new table name
