# smm_controllers

## Overview

`smm_controllers` is a ROS 2 control package dedicated to implementing **custom model-based controllers** for **Serial Metamorphic Manipulators (SMMs)**.

The package is designed to:

- Support **variable-structure robots (3–6 DOF)** generated at runtime
- Integrate tightly with:
  - `smm_screws` → kinematics & dynamics (screw theory-based)
  - `smm_synthesis` → runtime YAML-based robot generation
  - `smm_gazebo_sim` → physics-based validation in Gazebo
- Provide a **modular controller architecture** aligned with ROS 2 `ros2_control`

---

## Design Philosophy

This package follows a **layered and extensible architecture**:

- **Core Layer (`smm_controller_core`)**
  - Handles robot model loading via YAML
  - Interfaces with `RobotContextNdof`
  - Provides access to:
    - Kinematics (`ScrewsKinematicsNdof`)
    - Dynamics (`ScrewsDynamicsNdof`)

- **Controller Plugins**
  - Implemented as ROS 2 control plugins
  - Each controller:
    - Works with **runtime DOF (3–6)**
    - Uses **effort command interface**
    - Can be selected dynamically via launch

---

## Runtime Adaptation (Key Feature)

Unlike fixed manipulators, SMMs have **variable structure**.

Controllers in this package:

- Do **not assume a fixed number of joints**
- Receive joint names dynamically from:
  - `active_joint_names.yaml`
- Receive robot model data dynamically from:
  - synthesis-generated YAML files

This allows the **same controller implementation** to work across:

- Different anatomies
- Different DOFs
- Different synthesis results

---

## Implemented Controllers

### 1. Joint PD Effort Controller

**Plugin:** smm_controllers/JointPDEffortController

### Description

A baseline **joint-space proportional-derivative controller**:

\[
\tau = K_p (q_d - q) + K_d (\dot{q}_d - \dot{q})
\]

### Features

- Works in **pure effort control mode**
- Supports:
  - Runtime DOF (3–6)
  - Arbitrary joint sets
- Optional:
  - `hold_initial_position` → captures initial joint state as reference

### Parameters

```yaml
kp: [ ... ]   # per-joint proportional gains
kd: [ ... ]   # per-joint derivative gains
hold_initial_position: true
```

### 2. Joint PD + Gravity Effort Controller

**Plugin:** smm_controllers/PDGravityController

### Description

Extends the PD controller by adding **model-based gravity compensation**:

\[
\tau = K_p (q_d - q) + K_d (\dot{q}_d - \dot{q}) + G(q)
\]

### Features

- Works in **pure effort control mode**
- Supports:
  - Runtime DOF (3–6)
  - Arbitrary joint sets
  - Full robot model constructed from YAML, based on work implemented on `smm_synthesis`.
  - Integrates **ScrewsKinematics** and **ScrewsDynamics** from ros_pkg `smm_screws`.
- Computes:
  - Gravity vector in real-time.  
- Optional:
  - `hold_initial_position` → captures initial joint state as reference

### Parameters

```yaml
kp: [ ... ]   # per-joint proportional gains
kd: [ ... ]   # per-joint derivative gains
hold_initial_position: true
dynamics_data_dir: ~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml
gravity_representation: body
```

---

## Integration with Gazebo

Works along ros_pkg `smm_gazebo_sim`. Controllers are loaded dynamically via:

```text
spawn_smm_gazebo_control.launch.py
```
## Example usage:

```text
ros2 launch smm_gazebo_sim spawn_smm_gazebo_control.launch.py \
  controller_type:=pd_gravity
```

