# Benchmark Planning Servers

A typical planning problem consists of different types of subproblems. For example, a simple welding task could look like this:
```irl
commands
movej home
movep P1
movel P2
movej home`
```

Where `movej` and `movep` are free space motions to a given robot configuration or end-effector pose. The actual welding path is a straigh line Cartesian motion shown above as `movel` starting in pose `P1` going to pose `P2`.

 We define a standard interface to describe these subproblems in the ROS package [nexon_msgs](https://bitbucket.org/jeroendm/nexon/src/master/). This means the high-level planner can ignore the details of the algorithms that solve the subproblems and just use the solution. Concretly, the low-level planners publish a ROS service that the high-level planner can call.

The low level planner can be ROS independent, communicating with ROS through the JSON API of the [rosbridge_suite](https://wiki.ros.org/rosbridge_suite).
This makes it particulary easy to write planners in [javascript](https://github.com/RobotWebTools/roslibjs) or [Python(3)](https://roslibpy.readthedocs.io/en/latest/)

## Standard interface
A a low level planner can expose 3 types of interfaces:

### PTP planning

Point-to-point planning. Starting from a known configuration, the robot has to move to a goal configuration, or a goal end-effector pose (using inverse kinematics to find valid goal configurations.) The ros service definition looks like this:
`nexon_msgs/PTPPlanning.srv`
```
float64[] start_config
geometry_msgs/Pose pose_goal
float64[] joint_goal
---
bool success
trajectory_msgs/JointTrajectoryPoint[] joint_path
```

### Linear planning

Linear Cartesian motion. The robot starts from a known configuration and moves with it's end-effector in a straigh line to a goal pose.
In addition you can add pose constraints. This means that the robot does not have to move to the exact goal pose, but has to object the constraints given around this goal pose. The constraints are assumed to be valid along the entire linear Cartesian path.
`nexon_msgs/LINPlanning.srv`
```
float64[] start_config
geometry_msgs/Pose pose_goal
bool has_constraints
nexon_msgs/PoseConstraint pose_constraint
---
bool success
trajectory_msgs/JointTrajectoryPoint[] joint_path
```

### Follow Cartesian Path planning
For more complex paths (not a straight line) one can specify a list of desired end-effector pose, with or without constraints along the path.
`nexon_msgs/FLPlanning.srv`
```
nexon_msgs/CartesianPath path
nexon_msgs/JointPathPoint[] initial_guess
---
bool success
nexon_msgs/JointPathPoint[] joint_path
```

## Constraints

When one is bussy with arc welding, rotation around the torch does not influence the process. To specify such task constraints (actually, calling it tolerances makes more sense somethimes...) you can use `nexon_msgs/PoseConstraint.msg`:
```
bool relative
float64[] xyz_min
float64[] rpy_min
float64[] xyz_max
float64[] rpy_max
```
These are either expressed in a global `/world` frame (`relative=true`) or in the local path frame (`relative=false`).
Bounds are added on the end-effector's position (`xyz_min` and `xyz_max`), or orientation expressed as roll, pitch and yaw angles (`rpy_min` and `rpy_max`). (XYZ-Euler angles around moving axes.) If the lower bound equals the upper bound, no tolerance is added by the planner on this parameter (obviously).

## Currently implemented

**ompl_planning_server**
- `PTPPlanning` service called `/ompl_ptp_planning`
- `LINPlanning` service called `/ompl_lin_planning`

**arf_planning_server**
- `LINPlanning` service called `/arf_cart_planning` (TODO, rename to make it consistent.)

# LICENSE

The main license for this repository is the MIT License, except for two files.
- `arf_planning_server/include/arf_planning_server/machine_specs.h`
- `arf_planning_server/src/machine_specs.cpp`
These are copied from [ompl](https://github.com/ompl/ompl) and contain a BSD License statement.
