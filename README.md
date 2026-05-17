# smm_controllers

## Overview

A ROS 2 control package for custom effort-based controllers for **Serial Metamorphic Manipulators (SMMs)**.

The package currently provides joint-space controllers for synthesized SMM anatomies generated at runtime. It is designed to work with variable-DOF robots and runtime YAML model data produced by the SMM synthesis pipeline.

Current controllers:

- `JointPDEffortController`
- `PDGravityController`
- `InverseDynamicsJointController`

The package integrates with:

| Package | Role |
|---|---|
| `smm_synthesis` | Generates the SMM anatomy, URDF/Xacro, active joint list, and runtime YAML data |
| `smm_screws` | Provides screw-theory-based kinematics and dynamics |
| `smm_gazebo_sim` | Spawns the synthesized SMM in Gazebo and loads the selected controller |
| `smm_controllers` | Provides custom `ros2_control` controller plugins |

---

## Main Scope

Provides a reusable controller layer for SMM robots with:

- runtime DOF support,
- runtime active joint loading,
- effort-based command output,
- model-based dynamics support,
- Gazebo validation,
- common command interface,
- common debug/error topics,
- extensible structure for future Cartesian, robust, and adaptive controllers.

The package currently implements only **joint-space controllers**.

Under devel: **operational-space controllers**

Next: **robust-adaptive control**

---

## Runtime data

SMMs can change their structure and active anatomy. Therefore, the controllers do not hardcode joint names or DOF.

Joint names are loaded from runtime-generated folder data in:

```text
~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml
```

---

## Core API

All controllers are implemented as ROS 2 control plugins.

Standard lifecycle functions:
- on_init()
- on_configure()
- on_activate()
- update()

Standard interface functions:
- command_interface_configuration()
- state_interface_configuration()

---

## Shared Controller Helper Functions

Generic helper functions are placed in:

```text
include/smm_controllers/core/smm_controller_utils.hpp
src/core/smm_controller_utils.cpp
```

Current shared helper functions:

- make_interface_names()
- write_vector_to_command_interfaces()
- fill_joint_error_state_msg()
- publish_scalar_error_topics()

Trajectory-related helper functions in :

```text
include/smm_controllers/core/trajectory_operation_utils.hpp
src/core/trajectory_operation_utils.cpp
```

Current interpolation modes:

- sync_cubic
- cubic_hermite

---

## Command Interface

All current custom controllers support a single joint-space command through:

```text
/smm_joint_controller/reference
```

Message type:

```text
trajectory_msgs/msg/JointTrajectory
```

For `JointPDEffortController` and `PDGravityController`, only the first trajectory point is currently used as a single desired joint state.

For `InverseDynamicsJointController`, both single-point commands and multi-point trajectories are supported.

---

## Desired State Topics

Full desired joint state:

```text
/smm_joint_controller/desired_joint_state
```

Message type:

```text
sensor_msgs/msg/JointState
```
---

---

## Debug Topics

Full error state:

```text
/smm_joint_controller/error_state
```

Message type:

```text
sensor_msgs/msg/JointState
```
---

## SMM Dynamics Adapter

The SmmDynamicsAdapter is the bridge between the controller plugins and the SMM dynamic model implemented in `smm_screws`. The developed model-based controllers call the adapter through a clean interface.

Main responsibilities:

- Load the runtime robot model from synthesis-generated YAML files.
- Initialize the SMM screw-theory context.
- Provide access to full joint-space dynamics.
- Hide internal model representation details from the controllers.
- Keep model-based controllers cleaner and easier to maintain.

The adapter is currently used by:

- PDGravityController
- InverseDynamicsJointController

### `SmmDynamicsAdapter` API Summary

| Function | Description |
|---|---|
| `initialize()` | Loads runtime YAML model and initializes the dynamics context. |
| `dof()` | Returns the active model degrees of freedom. |
| `computeGravity()` | Computes the joint-space gravity torque vector. |
| `computeJointDynamics()` | Computes mass matrix, Coriolis matrix, and gravity vector. |
| `parseRepresentation()` | Converts representation string to dynamics representation enum. |
| `createRobotFromYaml()` | Builds the SMM robot model from runtime YAML files. |
| `countPseudojointsFromAssembly()` | Counts valid pseudojoints from the assembly YAML file. |
| `robot_context_ndof_` | Owns the runtime SMM robot context. |
| `dof_` | Stores the active robot degrees of freedom. |
| `gravity_representation_` | Stores the selected dynamics representation. |
| `q_float_` | Float copy of joint positions for `smm_screws`. |
| `qdot_float_` | Float copy of joint velocities for `smm_screws`. |
| `qddot_zero_` | Zero acceleration vector for gravity/dynamics calls. |
| `dynamics_representation_str_` | Stores the selected representation as text. |
| `body_frame_selection_str_` | Stores the selected body-frame convention. |

## Implemented Controllers

### 1. Joint PD Effort Controller

Developed as simple startup. Nothing special here!

#### Default YAML Parameters

```text
joint_pd_effort:
  kp: 80.0
  kd: 10.0
  hold_initial_position: true
  q_des: []
  qdot_des: []
  publish_error_state: true
```

- `kp`: proportional gain (assigned to all joints)
- `kd`: derivative gain (assigned to all joints)
- `hold_initial_position`: if true, the current joint positions are captured as the initial reference
- `q_des`: (optional) desired joint positions
- `qdot_des`: (optional) desired joint velocities
- `publish_error_state`: enables debug/error topics

### 2. Joint PD+Gravity Controller

Dynamics computed from `smm_screws` comew into the game. But still nothing special here!

#### Plugin

```text
smm_controllers/PDGravityController
```
#### Adapter Use

```text
dynamics_adapter_.computeJointDynamics(
  q_,
  qdot_,
  mass_matrix_,
  coriolis_matrix_,
  gravity_);
  ```
Both model-based controllers initialize the adapter during `on_configure()`!

```text
Both model-based controllers initialize the adapter during on_configure().
```

`PDGravityController`:
```text
dynamics_adapter_.initialize(dynamics_data_dir_, gravity_representation_)
```
`InverseDynamicsJointController`:
```text
dynamics_adapter_.initialize(dynamics_data_dir_, dynamics_representation_)
```

#### How to test:

```text
ros2 launch smm_gazebo_sim spawn_smm_gazebo_control.launch.py   controller_type:=pd_gravity   start_rqt_plot:=false

ros2 run smm_controllers send_joint_goal_error_fdbk.py   --active-joint-names-yaml ~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml/active_joint_names.yaml   --goal-yaml ~/ros2_ws/src/smm_class_pkgs/smm_control/smm_controllers/config/manual_commands/joint_goal_6dof.yaml   --goal small_joint2_test   --topic /smm_joint_controller/reference   --controller-name smm_joint_controller   --publish-rate-hz 5.0   --publish-duration-sec 5.0   --error-tolerance 0.001   --stable-time-sec 1.0   --timeout-sec 8.0
```

### 3. Inverse-Dynamics Joint-Space Controller

#### Plugin

```text
smm_controllers/InverseDynamicsJointController
```

Main implementation points:

- reads position and velocity state interfaces,
- supports hold, single-point, and trajectory reference modes,
- samples multi-point joint trajectories,
- computes the stabilized acceleration term,
- obtains mass matrix, Coriolis matrix, and gravity vector through `SmmDynamicsAdapter`,
- writes effort commands to the hardware interface,
- publishes error/debug topics.

The controller currently supports:

```text
HOLD
SINGLE_POINT
TRAJECTORY
```

#### ROS 2 Controller API:

| Function | Description |
|---|---|
| `command_interface_configuration()` | Declares required effort command interfaces for active joints. |
| `state_interface_configuration()` | Declares required position and velocity state interfaces. |
|`on_init()`|Declares controller parameters and default values.|
| `on_configure()` | Loads parameters, initializes vectors, publishers, subscriber, and dynamics adapter.|
| `on_activate()` | Validates interfaces and optionally captures the initial joint position. |
| `update()` | Executes the real-time control loop. |
| `referenceCallback()` | Receives and buffers incoming `JointTrajectory` reference commands. |
| `processReferenceCommandIfAvailable()` | Reads buffered commands and updates controller reference mode. |
| `acceptSinglePointCommand()` | Accepts one-point joint references and holds the target. |
| `startTrajectoryCommand()` | Starts execution of a multi-point joint trajectory. |
| `sampleActiveTrajectory()` | Samples the active trajectory and updates desired states. |
| `validateTrajectoryMessage()` | Checks reference message size and timing consistency. |
| `readStateInterfaces()` | Reads current joint positions and velocities. |
| `computeInverseDynamicsCommand()` | Computes inverse-dynamics torque command. |
| `publishDebugState()` | Publishes joint error and torque debug data. |
| `writeCommandInterfaces()` | Writes computed torques to effort command interfaces. |

#### How to test:

```text
ros2 launch smm_gazebo_sim spawn_smm_gazebo_control.launch.py \
  controller_type:=joint_inverse_dynamics \
  start_rqt_plot:=false
  
ros2 launch smm_controllers send_joint_goal_with_plot.launch.py

ros2 run smm_controllers send_joint_trajectory.py \
  --active-joint-names-yaml ~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml/active_joint_names.yaml \
  --trajectory-yaml ~/ros2_ws/src/smm_class_pkgs/smm_control/smm_controllers/config/manual_commands/joint_trajectory_6dof.yaml \
  --trajectory joint2_cubic_test \
  --publish-count 1
```
