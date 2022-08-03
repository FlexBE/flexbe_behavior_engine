# FlexBE Behavior Engine

FlexBE is a high-level behavior engine coordinating the capabilities of a robot in order to solve complex tasks.
Behaviors are modeled as hierarchical state machines where states correspond to active actions and transitions describe the reaction to outcomes.
Main advantage over similar approaches is the good operator integration and an
intuitive user interface.
Besides executing behaviors in full autonomy, the operator can restrict execution of certain transitions or trigger them manually.
Furthermore, it is even supported to modify the whole structure of a behavior during its execution without restarting it.
The user interface features a runtime control interface as well as a graphical editor for state machines.

Please refer to the FlexBE Homepage ([flexbe.github.io](http://flexbe.github.io)) for further information, tutorials, application examples, and much more.

![FlexBE CI](https://github.com/FlexBE/flexbe_behavior_engine/workflows/FlexBE%20CI/badge.svg)

## Installation

Execute the following commands to install FlexBE for ROS 2 systems:

    cd "ros2_ws"/src
    git clone https://github.com/team-vigir/flexbe_behavior_engine.git

Next, navigate to the "ros2_ws" top-level directory and build FlexBE:

    colcon build

Furthermore, create your own repository for behavior development (contains examples):

    ros2 run flexbe_widget create_repo [your_project_name]

Finally, it is recommended to install the FlexBE App user interface by following [these steps](http://philserver.bplaced.net/fbe/download.php).

## Usage

Use the following launch file for running the onboard engine:

    ros2 launch flexbe_onboard behavior_onboard.launch.py

Use the following launch file for running the operator control station (requires the FlexBE App):

    ros2 launch flexbe_app flexbe_ocs.launch.py

Use the following launch file to run both of the above, e.g., for testing on a single computer:

    ros2 launch flexbe_app flexbe_full.launch.py

For running test use:
python -m unittest src/flexbe_behavior_engine/<flexbe_dir>/test/<test.py>

## Next Steps

- Do some of the [tutorials](http://philserver.bplaced.net/fbe/documentation.php).
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

## Maintainer

- Philipp Schillinger ([@pschillinger](https://github.com/pschillinger), [Contact](http://philserver.bplaced.net/fbe/contact.php))
