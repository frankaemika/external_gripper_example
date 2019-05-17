#!/usr/bin/env python

import actionlib_msgs.msg
import franka_control.srv
import geometry_msgs.msg
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction, CommandRobotiqGripperGoal, CommandRobotiqGripperResult
import controller_manager_msgs.srv
import moveit_commander
import actionlib
import rospy
import rospkg
import rosparam
import os.path
import sys

node_prefix = 'panda_with_robotiq_gripper_example: '  # To label log messages coming from this file.
timeout = 10.0  # [s] Time to wait for services actions and topic to become available.


class Context:
    '''Class that ensures that all services action and topics needed for the application are loaded.
    '''

    def __init__(self, move_group_name):
        ''' Initialized the Context class by connecting to all services, actions and the movegroup.
        :param move_group_name Name of the movegroup to use for motion planning.
        '''
        global node_prefix, timeout
        rospy.loginfo(node_prefix + 'Waiting for move_group/status')
        try:
            rospy.wait_for_message('move_group/status',
                                   actionlib_msgs.msg.GoalStatusArray, timeout)
        except:
            rospy.logerr(node_prefix +
                         'Timeout waiting for required topic. Shutting down!')
            sys.exit(1)

        self.commander = moveit_commander.MoveGroupCommander(move_group_name)
        rospy.loginfo(node_prefix +
                      'Waiting for robotiq_gripper/command_robotiq_action')
        self.gripper_move = actionlib.SimpleActionClient(
            '/robotiq_gripper/command_robotiq_action',
            CommandRobotiqGripperAction)
        if not self.gripper_move.wait_for_server(rospy.Duration(timeout)):
            rospy.logerr(node_prefix +
                         'Timeout waiting for required action. Shutting down!')
            sys.exit(1)

        rospy.loginfo(node_prefix +
                      'Waiting for controller_manager/switch_controller')
        self.switch_controller = rospy.ServiceProxy(
            'controller_manager/switch_controller',
            controller_manager_msgs.srv.SwitchController)
        try:
            self.switch_controller.wait_for_service(10)
        except:
            rospy.logerr(
                node_prefix +
                'Timeout waiting for required service. Shutting down!')
            sys.exit(1)

        rospy.loginfo(node_prefix + 'Waiting for '
                      'franka_control/set_force_torque_collision_behavior')
        self.set_collision_behavior = rospy.ServiceProxy(
            'franka_control/set_force_torque_collision_behavior',
            franka_control.srv.SetForceTorqueCollisionBehavior)
        try:
            self.set_collision_behavior.wait_for_service()
        except:
            rospy.logerr(
                node_prefix +
                'Timeout waiting for required service. Shutting down!')
            sys.exit(1)

        self.active_controllers = []

    def load_controllers(self, controllers):
        '''Loads a set of controllers to control the Panda.
        :param: controllers The set of controllers to load.
        '''
        global node_prefix
        params = controller_manager_msgs.srv.SwitchControllerRequest()
        params.start_controllers = controllers
        params.stop_controllers = self.active_controllers
        params.strictness = params.STRICT
        if not self.switch_controller(params):
            rospy.logerr(node_prefix + 'Couldn\'t switch controllers')
            sys.exit(1)
        self.active_controllers = controllers


def moveit_joint(ctx, position, acc=0.1, vel=0.1):
    '''Performs a joint motion with moveit.
    :param ctx A Context class instance.
    :param position The joint pose target.
    :param acc a scaling factor for optionally reducing the maximum joint acceleration (0,1]
    :param vel a scaling factor for optionally reducing the maximum joint velocity (0,1]
    '''
    global node_prefix
    ctx.load_controllers(['position_joint_trajectory_controller'])
    rospy.loginfo(node_prefix + 'Moving to joint position {}'.format(position))
    ctx.commander.set_max_acceleration_scaling_factor(acc)
    ctx.commander.set_max_velocity_scaling_factor(vel)
    ctx.commander.go(position, wait=True)
    ctx.commander.stop()


def moveit_cart(ctx, pos, rot, acc=0.1, vel=0.1):
    '''Performs a Cartesian motion with moveit.
    :param ctx A Context class instance.
    :param position The Cartesian pose target.
    :param acc a scaling factor for optionally reducing the maximum joint acceleration (0,1]
    :param vel a scaling factor for optionally reducing the maximum joint velocity (0,1]
    '''
    global node_prefix
    ctx.load_controllers(['position_joint_trajectory_controller'])
    rospy.loginfo(node_prefix +
                  'Moving to Cartesian pose: pos {}, rot {}'.format(pos, rot))
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = rot[0]
    pose_goal.orientation.x = rot[1]
    pose_goal.orientation.y = rot[2]
    pose_goal.orientation.z = rot[3]
    pose_goal.position.x = pos[0]
    pose_goal.position.y = pos[1]
    pose_goal.position.z = pos[2]
    ctx.commander.set_max_acceleration_scaling_factor(acc)
    ctx.commander.set_max_velocity_scaling_factor(vel)
    ctx.commander.set_end_effector_link('panda_link8')
    ctx.commander.set_pose_target(pose_goal)
    ctx.commander.go(wait=True)
    ctx.commander.stop()
    ctx.commander.clear_pose_targets()


def gripper_move(ctx, position, speed, force):
    ''' Moves the gripper to a target position (width).
    :param ctx A Context class instance.
    :param position The target width. [m]
    :param speed The target end-effector speed for the motion. [m/s]
    :param force The force to apply. [N]
    '''
    global node_prefix
    goal = CommandRobotiqGripperGoal(position=position,
                                     speed=speed,
                                     force=force)
    rospy.loginfo('Moving gripper:\n{}'.format(goal))
    ctx.gripper_move.send_goal(goal)
    ctx.gripper_move.wait_for_result()
    result = ctx.gripper_move.get_result()
    if not abs(result.requested_position - result.position) < 0.001:
        rospy.logerr(node_prefix + 'Couldn\'t move gripper')
        sys.exit(1)


def set_collision_behavior(ctx, torques, forces):
    '''Configures the collision reflex thresholds of the Panda robot.
    :param torques The torque thresholds for all the 7 joints. [Nm]
    :param forces The Cartesian force thresholds. [N]
    '''
    global node_prefix
    rospy.loginfo(
        node_prefix +
        'Setting CB:\n torques: {}\nforces: {}'.format(torques, forces))
    ctx.set_collision_behavior(lower_torque_thresholds_nominal=torques,
                               upper_torque_thresholds_nominal=torques,
                               lower_force_thresholds_nominal=forces,
                               upper_force_thresholds_nominal=forces)


'''The supported action steps to compose action sequences.'''
STEPS = {
    'moveit_cart': moveit_cart,
    'moveit_joint': moveit_joint,
    'gripper_move': gripper_move,
    'set_collision_behavior': set_collision_behavior,
}


def create_step(t, params):
    '''helper method for creating a sequence step.
    :param t step index.
    :param params Parameter string for the step.
    '''
    callback = STEPS[t]
    return lambda ctx: callback(ctx, **params)


if __name__ == '__main__':
    # Initialize the node.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('icra19_panda_with_robotiq_gripper_example')

    # Wait for the hardware node to come up.
    rospy.loginfo(node_prefix +
                  'Waiting for controller_manager/load_controller')
    load_controller = rospy.ServiceProxy(
        'controller_manager/load_controller',
        controller_manager_msgs.srv.LoadController)
    try:
        load_controller.wait_for_service(timeout)
    except:
        rospy.logerr(node_prefix +
                     'Timeout waiting for controller service. Shutting down!')
        sys.exit(1)

    # Parse the controllers parameter to configure the controllers to run.
    if not rospy.has_param('~controllers'):
        rospy.logerr(
            node_prefix +
            'Could not find required parameter \'controllers\'. Aborting!')
        sys.exit(0)
    for controller_name in rospy.get_param('~controllers'):
        if not load_controller(controller_name):
            rospy.logerr(node_prefix + 'Could not load {}', controller_name)
            sys.exit(1)
    rospy.loginfo(node_prefix + 'Loaded controllers')

    # Initialize the Context class instance.
    rospack = rospkg.RosPack()
    ctx = Context('panda_arm')

    # Load the sequence steps to execute from the yaml file.
    demo_files = [
        os.path.join(rospack.get_path('panda_with_robotiq_gripper_example'),
                     'config', '{}.yaml'.format(f))
        for f in rospy.get_param('~demos')
    ]
    configs = [rosparam.load_file(f)[0][0] for f in demo_files]
    steps = [create_step(x['type'], x['params']) for x in sum(configs, [])]

    # Execute the steps.
    rospy.loginfo(node_prefix + 'Running steps')
    while not rospy.is_shutdown():
        for step in steps:
            if rospy.is_shutdown():
                sys.exit(0)
            step(ctx)
        rospy.sleep(2)
        raw_input(
            '-------------------------------\n' + node_prefix +
            'Press Enter to repeat the sequence..\n-------------------------------\n'
        )
    rospy.loginfo(node_prefix + 'Shutting down.')
