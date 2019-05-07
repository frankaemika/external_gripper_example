# icra18-fci-tutorial
This repository provides exemplary code to run a sequence of actions with Panda using `franka_ros`
(e.g. set the robot's collision behavior, reach a Cartesian pose with MoveIt!, run a custom controller, open the gripper, ...).
It also provides code for a simple Cartesian impedance controller that wiggles around a specific pose while performing force control.
This code was shown during the Panda tutorial at ICRA 2018 in Brisbane, Australia to plug-in PC connectors in a docking station.

## Quickstart

Requires `libfranka` >= 0.5.0, `franka_ros` >= 0.6.0.

To configure:

 1. Set all used controllers in the `launch/demo.launch` rosparam "controllers".
 2. Add the subtasks you want to perform in the `launch/demo.launch` rosparam "demos".
 3. Each of the subtasks are described in a YAML file `config/my_step.yaml` as a sequence of steps.
 The `scripts/demo.py` script will process them. It supports 6 possible steps:
    * `moveit_cart`: go to a specific Cartesian pose using MoveIt!.
    * `moveit_joint`: go to a specific joint configuration using MoveIt!.
    * `gripper_move`: move the gripper to a specific gripper width with the `franka::Gripper::move` command.
    * `gripper_grasp`: grasp an object with the `franka::Gripper::grasp` command.
    * `run_controller`: run a custom controller. In the provided examples it will be the included plug_in_controller.
    * `set_collision_behavior`: set the collision behavior for the robot.

To execute:

```
roslaunch icra18 demo.launch robot_ip:=<robot-ip>
```

You can also test just the plug_in controller an explore different configurations with rqt_reconfigure by running

```
roslaunch icra18 plug_in_controller.launch robot_ip:=<robot-ip>
```

To get specific parameters for your custom `moveit_cart`, `gripper_move` or `gripper_grasp` steps you might find useful running the teaching launch file, which constantly prints the robot pose and the gripper width, by executing

```
roslaunch icra18 teaching.launch robot_ip:=<robot-ip>
```
