# effibote3_description

## 1) Overview

`effibote3_description` provides the robot-specific description layer for the Effibot E3 mobile base.

It extends `romea_mobile_base_description` with the concrete configuration, URDF/Xacro files, meshes and `ros2_control` descriptions required to instantiate the Effibot E3 robot.

The Effibot E3 mobile base uses:

| Configuration file | Mobile base architecture | Command type |
|---|---|---|
| `config/effibote3.yaml` | `4WD` | `skid_steering` |

This architecture has four driving wheels and no steering joints. It is controlled as a skid-steering mobile base.

## 2) Robot configuration

The `config/` directory contains:

| File | Purpose |
|---|---|
| `effibote3.yaml` | full Effibot E3 mobile base configuration |
| `teleop.yaml` | default skid-steering teleoperation configuration |

The robot configuration follows the structure defined by `romea_mobile_base_description` and contains:

* the mobile base architecture (`4WD`);
* geometry, wheel dimensions and chassis bounding box;
* wheel speed command and feedback information;
* inertia and control point;
* link and joint names used in the URDF and `ros2_control` descriptions.

## 3) URDF and ros2_control descriptions

The URDF description is built from:

| Path | Role |
|---|---|
| `urdf/effibote3.urdf.xacro` | main URDF entry point |
| `urdf/effibote3.xacro` | Effibot E3 mobile base macro |
| `urdf/effibote3.simulation.xacro` | simulator-specific Gazebo or Gazebo Classic plugin insertion |
| `urdf/visual/` | visual Xacro fragments for chassis and wheels |
| `meshes/` | Effibot E3 chassis meshes |

The Effibot E3 macro reuses the `base4WD.chassis.xacro` template from `romea_mobile_base_description` and specializes it with Effibot E3 geometry, link names, joint names and visual meshes.

The `ros2_control` description is built from:

| Path | Role |
|---|---|
| `ros2_control/effibote3.ros2_control.urdf.xacro` | main `ros2_control` entry point |
| `ros2_control/effibote3.ros2_control.xacro` | Effibot E3 `ros2_control` macro |

Depending on the selected mode, the `ros2_control` description selects:

| Mode | Hardware plugin |
|---|---|
| `live` | `effibote3_hardware/EffibotE3Hardware` |
| `simulation`, `simulation_gazebo_classic` | `romea_mobile_base_gazebo/GazeboSystemInterface4WD` |
| `simulation_gazebo` | `romea_mobile_base_gazebo/GazeboSystemInterface4WD` |

## 4) Python API

The installed Python module provides helper functions used by `effibote3_bringup` and by the meta-bringup workflow.

| Function | Purpose |
|---|---|
| `get_specifications_path_file()` | returns the Effibot E3 configuration file path |
| `get_specifications_configuration()` | loads the full robot configuration |
| `get_configuration()` | returns the compact mobile base configuration completed with manufacturer, model and version |
| `generate_configuration_file(configuration, extended)` | serializes the compact configuration |
| `generate_urdf_description(...)` | generates the Effibot E3 URDF description |
| `generate_ros2_control_description(...)` | generates the Effibot E3 `ros2_control` description |

Example:

```python
from effibote3_description import get_configuration

configuration = get_configuration()
```

When `mode` is set to `simulation`, the Python API maps it to `simulation_gazebo_classic` before generating the URDF or `ros2_control` description.

## 5) Relation with other packages

`effibote3_description` is the Effibot E3 specialization of `romea_mobile_base_description`:

* `romea_mobile_base_description` provides the generic `4WD` description and `ros2_control` templates;
* `effibote3_description` provides the Effibot E3 configuration, meshes and Xacro specialization;
* `effibote3_hardware` provides the live `ros2_control` hardware plugin;
* `effibote3_bringup` uses this package to generate configuration, URDF and `ros2_control` artifacts for live and simulation modes.
