# FlexBE Behavior Engine

FlexBE is a high-level behavior engine coordinating the capabilities of a robot in order to solve complex tasks.
Behaviors are modeled as hierarchical state machines (HFSM) where states correspond to active actions
and transitions describe the reaction to outcomes.

The main advantage of FlexBE over similar approaches is the good operator integration and an intuitive user interface.
Besides executing behaviors in full autonomy, the operator can restrict execution of certain transitions or trigger them manually.
Furthermore, FlexBE supports modifying the whole structure of a behavior during its execution without restarting it.
The user interface features a runtime control interface as well as a graphical editor for state machines.

Please refer to the FlexBE Homepage ([flexbe.github.io](http://flexbe.github.io)) for further information, tutorials, application examples, and much more.

You may also want to check out the quick start tutorial demonstrations at [FlexBE Turtlesim Demo](https://github.com/FlexBE/flexbe_turtlesim_demo).

![FlexBE CI](https://github.com/FlexBE/flexbe_behavior_engine/workflows/FlexBE%20CI/badge.svg?branch=ros2-devel)

Jazzy ![ROS Build Farm](https://build.ros2.org/job/Jdev__flexbe_behavior_engine__ubuntu_noble_amd64/badge/icon)

Rolling ![ROS Build Farm](https://build.ros2.org/job/Rdev__flexbe_behavior_engine__ubuntu_noble_amd64/badge/icon)

> Note: This version 4+ breaks compatibility with the FlexBE App.  You must use the FlexBE WebUI [flexbe_webui](https://github.com/FlexBE/flexbe_webui.git) now.


## Installation

For released versions, FlexBE is available as `apt install` package `ros-<DISTRO>-flexbe-*`

To build from source, execute the following commands to install FlexBE for ROS 2 systems:

  `cd "ros2_ws"/src`

  `git clone https://github.com/FlexBE/flexbe_behavior_engine.git`

Next, navigate to the "ros2_ws" top-level directory and build FlexBE:

  `colcon build`

## Creating new FlexBE Behavior packages

To begin, create your own repository for behavior development in the `${WORKSPACE_ROOT}/src` folder:

  `ros2 run flexbe_widget create_repo [your_project_name] <meta_package_name> <--non-interactive>`

This will clone a project template (requires internet access) that contains examples and proper package definitions,
and create the ROS 2 package structure and three subfolders.

For example, running

  `ros2 run flexbe_widget create_repo my_project my_flexbe_project`

from the `${WORKSPACE_ROOT}/src` folder will create:
  - `${WORKSPACE_ROOT}/src/my_flexbe_project`
  - `${WORKSPACE_ROOT}/src/my_flexbe_project/my_flexbe_project` - the ROS meta package
  - `${WORKSPACE_ROOT}/src/my_flexbe_project/my_project_flexbe_behaviors`
  - `${WORKSPACE_ROOT}/src/my_flexbe_project/my_project_flexbe_states`

These are intended to contain your custom FlexBE state implementations and HFSM-based behaviors.

This release of the FlexBE Behavior Engine requires version 4.1+ of the FlexBE UI.
This breaks compatibility with the older FlexBE App and now requires use of the FlexBE WebUI tool.

It is recommended to install the FlexBE WebUI user interface:

  [FlexBE WebUI](https://github.com/FlexBE/flexbe_webui.git) - Python-based webserver version


## Usage

Use the following launch file for running the onboard engine:

  `ros2 launch flexbe_onboard behavior_onboard.launch.py`

Use the following launch file for running the operator control station (requires the FlexBE App or WebUI):

  `ros2 launch flexbe_webui flexbe_ocs.launch.py`

During testing is is recommended to start the base nodes and the UI client separately:

  `ros2 launch flexbe_webui flexbe_ocs.launch.py headless:=True`

  `ros2 run flexbe_webui webui_client`

  See the `flexbe_webui`  README for more details.


Use the following launch file to run the entire FlexBE system, both onboard and OCS, e.g., for testing on a single computer:

  `ros2 launch flexbe_webui flexbe_full.launch.py`

For running tests use:

  `colcon test --ctest-args --packages-select <flexbe_package>`

## Next Steps

- Do some of the [tutorials](http://flexbe.github.io).
- Visit the [FlexBE GitHub Organization](https://github.com/FlexBE) for additional available states.

- If you are converted an existing system from ROS 1 to ROS 2, we provide a
[ROS 2 Conversion Best Practices Guide](flexbe_states/ros2-conversion-best-practices.md)
to assist in converting any custom FlexBE state implementations.


## Publications

Please use the following publications for reference when using FlexBE:

- Philipp Schillinger, Stefan Kohlbrecher, and Oskar von Stryk, ["Human-Robot Collaborative High-Level Control with Application to Rescue Robotics"](http://dx.doi.org/10.1109/ICRA.2016.7487442), IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

- Joshua Zutell, David C. Conner and Philipp Schillinger, ["ROS 2-Based Flexible Behavior Engine for Flexible Navigation ,"](http://dx.doi.org/10.1109/SoutheastCon48659.2022.9764047), IEEE SouthEastCon, April 2022.

### Further Publications

Refer to the following publications to get an impression about ways to use FlexBE.
Let us know if you know a paper which should be added to the list.

- Stefan Kohlbrecher et al. ["A Comprehensive Software Framework for Complex Locomotion and Manipulation Tasks Applicable to Different Types of Humanoid Robots."](http://dx.doi.org/10.3389/frobt.2016.00031) Frontiers in Robotics and AI 3 (2016): 31.

- Spyros Maniatopoulos, Philipp Schillinger, Vitchyr Pong, David C. Conner, and Hadas Kress-Gazit, ["Reactive High-level Behavior Synthesis for an Atlas Humanoid Robot"](http://dx.doi.org/10.1109/ICRA.2016.7487613), IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

- Alberto Romay et al., [“Collaborative autonomy between high-level behaviors and human operators for remote manipulation tasks using different humanoid robots,”](http://dx.doi.org/10.1002/rob.21671) Journal of Field Robotics, September 2016.

- David C. Conner and Justin Willis, ["Flexible Navigation: Finite state machine-based integrated navigation and control for ROS enabled robots,"](http://dx.doi.org/10.1109/SECON.2017.7925266) SoutheastCon 2017.

## Maintainers

- Philipp Schillinger ([@pschillinger](https://github.com/pschillinger), [Contact](http://philserver.bplaced.net/fbe/contact.php))
- David Conner ([@dcconner](https://github.com/dcconner)) [Contact](https://flexbe.readthedocs.io/en/latest/contactinfo.html)
