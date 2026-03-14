# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

Always source ROS 2 and the workspace install before running commands:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Build:**
```bash
colcon build --symlink-install
```

**Build a single package:**
```bash
colcon build --symlink-install --packages-select <package_name>
```

**Run tests:**
```bash
colcon test
colcon test --packages-select <package_name>
colcon test-result --verbose  # view results
```

**Launch the full simulation (peg-in-hole task with UR5e):**
```bash
ros2 launch peg_in_hole_task ur_with_task.launch.py ur_type:=ur5e
```

**Run MoveIt motion planning demo:**
```bash
ros2 run moveit_playground pose_goal_demo
```

## Architecture

Three packages in `src/`:

### `Universal_Robots_ROS2_Gazebo_Simulation/` (upstream, do not modify)
Provides Gazebo simulation for UR robots. Key launch files:
- `ur_sim_control.launch.py` — starts Gazebo + ros2_control + robot controllers
- `ur_sim_moveit.launch.py` — starts MoveIt2 + RViz2 on top of the simulation

### `peg_in_hole_task/` (Python, ament_python)
Spawns peg and hole objects into the running Gazebo simulation. Entry point is `spawn_task_objects.py`, which reads spawn positions/orientations from `config/task_params.yaml`, converts RPY to quaternions, and calls the Gazebo `/spawn_entity` service. The composite launch file `ur_with_task.launch.py` includes both upstream launch files and then starts the spawner node.

Object spawn positions (in `config/task_params.yaml`):
- Hole block: XYZ `[-0.80, 0.00, 0.45]`, RPY `[0, π/2, 0]`
- Peg: XYZ `[0.45, 0.20, 0.01]`, RPY `[0, π/2, 0]`

### `moveit_playground/` (C++, ament_cmake)
MoveIt2 motion planning examples. `pose_goal.cpp` demonstrates planning to a Cartesian pose target using `MoveGroupInterface` with planning group `ur_manipulator` and end-effector `tool0`.

## Key Configuration

- **UR robot type** — passed as `ur_type` launch argument; supported: ur3, ur3e, ur5, ur5e, ur7e, ur10, ur10e, ur12e, ur16e, ur20, ur30. Default: `ur5e`.
- **Task object models** — SDF physics models in `src/peg_in_hole_task/models/` (peg: 0.2 kg box, 28×10×80 mm).
- **MoveIt planning parameters** in `pose_goal.cpp`: 5 s timeout, 5 planning attempts, 20% velocity/acceleration scaling.
