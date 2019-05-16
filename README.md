# icra19-gripper-example
This repository provides exemplary code to use panda with an external gripper for research purposes. The package allows the user to deploy a Pick-and-Place sequence for Panda with a mounted Robotiq 2F-85 Gripper.
It is demonstrated how Panda can be controlled via `franka_ros`
(e.g. set the robot's collision behavior, reach a Joint Configuration with MoveIt!) and how the Robotiq 2F-85 Gripper can be operated. This code is published referring to the ICRA 2019 in Montreal, Canada. 

## Presrequisites
The software package was successfully tested with the following dependencies
  
- `libfranka` == 0.5.0
- `franka_ros` == 0.6.0
- `robotiq_2finger_grippers` == 0.9.1
- `panda_moveit_config` == 0.7.2


TODO(jaeh_ch):

```shell
sudo usermod -a -G dialout <your_username>
sudo apt install ros-kinetic-libfranka
sudo apt install ros-kinetic-franka-ros
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/Danfoa/robotiq_2finger_grippers.git
git clone https://github.com/ros-planning/panda_moveit_config.git
git clone https://github.com/frankaemika/icra19_panda_with_robotiq_gripper_example.git
cd ..
source /opt/ros/kinetic/setup.bash
catkin_make
source devel/setup.bash
lsusb | grep robotiq # to find out to what USB Port the gripper is connected
roslaunch  icra19_panda_with_robotiq_gripper_example icra19_panda_with_robotiq_gripper_example.launch TODO: args 
```

For the installation of 'libfranka', 'franka_ros' and `panda_moveit_config` please refer to [Franka Control Interface Documentation](https://frankaemika.github.io/docs/overview.html). For the interaction with the Robotiq 2F-85 gripper the package `robotiq_2finger_grippers` must be cloned and built into the ROS workspace from:

```
https://github.com/Danfoa/robotiq_2finger_grippers
```

This repository was chosen for the control of the Robotiq gripper, because it provides a JointState Feedback of the gripper's joints facilitating a live visualization in RViz. Different implementations exist, like the [official ROS drivers from Robotiq](https://github.com/ros-industrial/robotiq)


## Deployment
 1. Start your Panda and change the End-Effector Settings according to the Robotiq 2F-85 gripper in `Desk`. See [Franka Control Interface Documentation](https://frankaemika.github.io/docs/overview.html) on how to accomplish this. For faster deployment you can also just `upload` predefined settings in `Desk` derived from the 2F-85 manual. The file is located here:

```
icra19/config/endeffector-config.json
```

 2. Connect the gripper to your computer. In our Setup the gripper was connected using the Robotiq K-1444 Universal Controller. See Robotiq's support page for further information.
TODO(jaeh_ch) add link!!!

 3. Launch the example launch file to execute the pick-and-place sequence passing your Panda's IP address and the USB device name of your Robotiq 2F-85 gripper (e.g.: "/dev/ttyUSB1"). Attention: In our example the gripper's driver provided to devices.

```
roslaunch icra19 demo.launch robot_ip:=<robot-ip> gripper_dev_name:=<gripper-dev-name>
```

## Customization
Each of the subsequential steps of the Pick and Place Demo are described in the YAML file. You can change them for your custom sequence.

```
icra19/config/pick_and_place.yaml
```

The `icra19/scripts/demo.py` script will process them. It supports 4 possible steps:
  
- `moveit_cart`: go to a specific Cartesian pose using MoveIt!.
- `moveit_joint`: go to a specific joint configuration using MoveIt!.
- `gripper_move`: move the Robotiq 2F-85 gripper to a specific gripper position between `0.0: closed` and `1.0: open`.
- `set_collision_behavior`: set the collision behavior for the robot.

