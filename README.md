# icra19-gripper-example
This repository provides exemplary code to use panda with an external gripper for research purposes. The package allows the user to deploy a Pick-and-Place sequence for Panda with a mounted Robotiq 2F-85 Gripper.
It is demonstrated how Panda can be controlled via `franka_ros`
(e.g. set the robot's collision behavior, reach a Joint Configuration with MoveIt!) and how the Robotiq 2F-85 Gripper can be operated. This code is published referring to the ICRA 2019 in Montreal, Canada. 

## Presrequisites
The software package was successfully tested with the following dependencies
- `libfranka` == 0.5.0
- `franka_ros` == 0.6.0
- `robotiq_2finger_grippers` == 0.9.1

For the installation of 'libfranka' and 'franka_ros' please refer to [Franka Control Interface Documentation](https://frankaemika.github.io/docs/overview.html). For the interaction with the Robotiq 2F-85 Gripper the package `robotiq_2finger_grippers` must be cloned into the ROS workspace from:

```
https://github.com/Danfoa/robotiq_2finger_grippers
```

This repository was chosen for the control of the Robotiq gripper, because it provides a JointState Feedback of the gripper's joints facilitating a live visualization in RViz. Different implementations exist, like the [official ROS drivers from Robotiq](https://github.com/ros-industrial/robotiq)


## Deployment
 1. Start your Panda and change the End-Effector Settings according to the Robotiq 2F-85 Gripper in `Desk`. See [Franka Control Interface Documentation](https://frankaemika.github.io/docs/overview.html) on how to accomplish this. For faster deployment you can also just `upload` predefined settings in `Desk` derived from the 2F-85 manual. The file is located here:

```
icra19/config/endeffector-config.json
```

 2. Launch the Demo Script to execute the examplary Pick-and-Place sequence passing your Panda's IP address.
```
roslaunch icra19 demo.launch robot_ip:=<robot-ip>
```

## Customization
Each of the subsequential steps of the Pick and Place Demo are described in the YAML file. You can change them for your custom sequence.

```
icra19/config/pick_and_place.yaml
```

The `icra19/scripts/demo.py` script will process them. It supports 4 possible steps:
    * `moveit_cart`: go to a specific Cartesian pose using MoveIt!.
    * `moveit_joint`: go to a specific joint configuration using MoveIt!.
    * `gripper_move`: move the Robotiq 2F-85 Gripper to a specific gripper position between `0.0: closed and `1.0: open`.
    * `set_collision_behavior`: set the collision behavior for the robot.

To get specific parameters for your custom `moveit_joint`, `moveit_cart` steps you might find useful running the teaching launch file, which constantly prints the robot pose and the gripper width, by executing

```
roslaunch icra18 teaching.launch robot_ip:=<robot-ip>
```
