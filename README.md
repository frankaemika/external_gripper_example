# icra19-gripper-example #

This repository provides example code on how Panda can be controlled via `franka_ros` in combination with an external gripper. In this particular example, the Robotiq 2F-85 gripper is used mounted on the flange of the Panda (see `panda_with_robotiq_gripper_example.png`). The package allows the user to run a configurable pick-and-place example sequence.
This software is tested with the following dependency version:

- `libfranka` == 0.5.0
- `franka_ros` == 0.6.0
- `robotiq_2finger_grippers` == 0.9.1
- `panda_moveit_config` == 0.7.2

For the installation of 'libfranka', 'franka_ros' and `panda_moveit_config` please refer to [Franka Control Interface Documentation](https://frankaemika.github.io/docs/overview.html).
In our example we use the following repo to control the Robotiq 2F-85 gripper:

```
https://github.com/Danfoa/robotiq_2finger_grippers
```

Note: The are other options of ros_packages for robotiq gripper like the [official ROS drivers from Robotiq](https://github.com/ros-industrial/robotiq).


# Deployment #

 1. Start your Panda and make sure that your end-effector settings match the Robotiq 2F-85 gripper's values in `Desk`. See [Franka Control Interface Documentation](https://frankaemika.github.io/docs/overview.html) on how to accomplish this. For faster deployment you can ``upload` predefined settings in `Desk` derived from the 2F-85 manual. The file is located here:

```
panda_with_robotiq_gripper_example/config/endeffector-config.json
```

 2. Connect the gripper to your pc via USB. In our setup we use e.g. a Robotiq K-1444 Universal Controller which is connected via USB to the control Pc. See Robotiq's support page for further information (https://robotiq.com/de/produkte/adaptiver-2-finger-robotergreifer-2f85-140).

 3. Make sure to install all dependencies, set up your workspace and build it:

 ```shell
sudo usermod -a -G dialout <your_username>  # Your user must be in the dialout group to connect to USB ports
sudo apt install ros-kinetic-libfranka      # If not already installed
sudo apt install ros-kinetic-franka-ros
mkdir -p catkin_ws/src                      # Create a folder for your workspace
cd catkin_ws/src
git clone https://github.com/Danfoa/robotiq_2finger_grippers.git
git clone https://github.com/ros-planning/panda_moveit_config.git
git clone https://github.com/frankaemika/icra19_gripper_example.git
cd ..
source /opt/ros/kinetic/setup.bash
catkin_make
source devel/setup.bash
```

 4. Now you can launch the example application with

```shell
roslaunch panda_with_robotiq_gripper_example panda_with_robotiq_gripper_example.launch robot_ip:=<robot-ip> gripper_dev_name:=<usb-bus-of-the-gripper>
```
Note: You need pass the USB-bus to which the gripper is connected to the launch file. You can find out where you connected the gripper with the `lsusb` command(e.g. "/dev/ttyUSB1").


# Customization #

Each of the sequential steps of the pick-and-place demo are described in the file:

```
icra19/config/pick_and_place.yaml
```

Feel free to customize them to configure your own sequence.
The `panda_with_robotiq_gripper_example/scripts/panda_with_robotiq_gripper_example.py` script will process them. It supports 4 possible steps:

- `moveit_cart`: Go to a specific Cartesian pose using MoveIt!.
- `moveit_joint`: Go to a specific joint configuration using MoveIt!.
- `gripper_move`: Move the Robotiq 2F-85 gripper to a specific gripper position between `0.0: closed` and `1.0: open`.
- `set_collision_behavior`: Set the collision behavior for the Panda robot.
