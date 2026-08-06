# effibote3

## Overview

`effibote3` groups the ROS2 packages that describe, launch and control the Effibot E3 mobile base in live and simulation modes.

This repository-level README gives a map of the stack. Detailed information about each package can be found in the corresponding package README.

## Packages

| Package | Role |
| --- | --- |
| `effibote3` | Metapackage that groups the Effibot E3 ROS2 packages. |
| `effibote3_description` | Robot-specific description layer for Effibot E3, including configuration files, URDF/Xacro descriptions, meshes and ros2_control descriptions. |
| `effibote3_bringup` | Main integration entry point for generating Effibot E3 configuration files, URDF descriptions, ros2_control descriptions and launch files. |
| `effibote3_hardware` | Live `ros2_control` hardware plugin for the Effibot E3 mobile base, built on the generic `4WD` hardware abstraction. |

## Usage

In most cases, start with `effibote3_bringup`. It is the user-facing entry point of the stack and the package used by `romea_mobile_base_meta_bringup` when an Effibot E3 model is selected from a mobile base meta-description.

The Effibot E3 stack is a robot-specific specialization of `romea_mobile_base`. The mobile base architecture is `4WD` and it is commanded as a skid-steering robot; `effibote3_description` provides the concrete geometry and generated descriptions, `effibote3_hardware` provides the live hardware implementation, and `effibote3_bringup` connects these pieces to the generic mobile base launch workflow.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `effibote3` project was developed by Jean Laneurit in the context of the BAUDETROB ANR project.

## Contact

For questions or comments about this project, please contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
