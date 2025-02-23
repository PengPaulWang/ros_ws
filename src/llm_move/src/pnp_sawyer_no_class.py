#!/usr/bin/env python

import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import intera_interface

def initialize_robot(limb="right", hover_distance=0.15, tip_name="right_gripper_tip"):
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()
    return {
        'limb_name': limb,
        'tip_name': tip_name,
        'hover_distance': hover_distance,
        'limb': intera_interface.Limb(limb),
        'gripper': intera_interface.Gripper(),
        'rs': rs,
        'init_state': init_state
    }

def move_to_start(robot, start_angles=None):
    print("Moving the {0} arm to start pose...".format(robot['limb_name']))
    if not start_angles:
        start_angles = dict(zip(robot['limb'].joint_names(), [0]*7))
    guarded_move_to_joint_position(robot, start_angles)
    gripper_open(robot)

def guarded_move_to_joint_position(robot, joint_angles, timeout=5.0):
    if rospy.is_shutdown():
        return
    if joint_angles:
        robot['limb'].move_to_joint_positions(joint_angles, timeout=timeout)
    else:
        rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

def gripper_open(robot):
    robot['gripper'].open()
    rospy.sleep(1.0)

def gripper_close(robot):
    robot['gripper'].close()
    rospy.sleep(1.0)

def approach(robot, pose):
    approach_pose = copy.deepcopy(pose)
    approach_pose.position.z += robot['hover_distance']
    joint_angles = robot['limb'].ik_request(approach_pose, robot['tip_name'])
    robot['limb'].set_joint_position_speed(0.001)
    guarded_move_to_joint_position(robot, joint_angles)
    robot['limb'].set_joint_position_speed(0.1)

def retract(robot):
    current_pose = robot['limb'].endpoint_pose()
    ik_pose = Pose()
    ik_pose.position.x = current_pose['position'].x
    ik_pose.position.y = current_pose['position'].y
    ik_pose.position.z = current_pose['position'].z + robot['hover_distance']
    ik_pose.orientation.x = current_pose['orientation'].x
    ik_pose.orientation.y = current_pose['orientation'].y
    ik_pose.orientation.z = current_pose['orientation'].z
    ik_pose.orientation.w = current_pose['orientation'].w
    servo_to_pose(robot, ik_pose)

def servo_to_pose(robot, pose, time=4.0, steps=400.0):
    r = rospy.Rate(1/(time/steps))
    current_pose = robot['limb'].endpoint_pose()
    ik_delta = Pose()
    ik_delta.position.x = (current_pose['position'].x - pose.position.x) / steps
    ik_delta.position.y = (current_pose['position'].y - pose.position.y) / steps
    ik_delta.position.z = (current_pose['position'].z - pose.position.z) / steps
    ik_delta.orientation.x = (current_pose['orientation'].x - pose.orientation.x) / steps
    ik_delta.orientation.y = (current_pose['orientation'].y - pose.orientation.y) / steps
    ik_delta.orientation.z = (current_pose['orientation'].z - pose.orientation.z) / steps
    ik_delta.orientation.w = (current_pose['orientation'].w - pose.orientation.w) / steps
    for d in range(int(steps), -1, -1):
        if rospy.is_shutdown():
            return
        ik_step = Pose()
        ik_step.position.x = d*ik_delta.position.x + pose.position.x
        ik_step.position.y = d*ik_delta.position.y + pose.position.y
        ik_step.position.z = d*ik_delta.position.z + pose.position.z
        ik_step.orientation.x = d*ik_delta.orientation.x + pose.orientation.x
        ik_step.orientation.y = d*ik_delta.orientation.y + pose.orientation.y
        ik_step.orientation.z = d*ik_delta.orientation.z + pose.orientation.z
        ik_step.orientation.w = d*ik_delta.orientation.w + pose.orientation.w
        joint_angles = robot['limb'].ik_request(ik_step, robot['tip_name'])
        if joint_angles:
            robot['limb'].set_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
        r.sleep()
    rospy.sleep(1.0)

def pick(robot, pose):
    if rospy.is_shutdown():
        return
    gripper_open(robot)
    approach(robot, pose)
    servo_to_pose(robot, pose)
    if rospy.is_shutdown():
        return
    gripper_close(robot)
    retract(robot)

def place(robot, pose):
    if rospy.is_shutdown():
        return
    approach(robot, pose)
    servo_to_pose(robot, pose)
    if rospy.is_shutdown():
        return
    gripper_open(robot)
    retract(robot)

def load_gazebo_models(table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725)),
                       block_reference_frame="world"):
    model_path = rospkg.RosPack().get_path('sawyer_sim_examples')+"/models/"
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')
    block_xml = ''
    with open(model_path + "block/model.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException as e:
        print("Delete Model service call failed: {0}".format(e))

def main():
    rospy.init_node("ik_pick_and_place_demo")
    load_gazebo_models()
    rospy.on_shutdown(delete_gazebo_models)

    limb = 'right'
    hover_distance = 0.15
    starting_joint_angles = {'right_j0': -0.041662954890248294,
                             'right_j1': -1.0258291091425074,
                             'right_j2': 0.0293680414401436,
                             'right_j3': 2.17518162913313,
                             'right_j4': -0.06703022873354225,
                             'right_j5': 0.3968371433926965,
                             'right_j6': 1.7659649178699421}
    robot = initialize_robot(limb, hover_distance)
    overhead_orientation = Quaternion(
        x=-0.00142460053167,
        y=0.999994209902,
        z=-0.00177030764765,
        w=0.00253311793936)
    block_poses = [
        Pose(
            position=Point(x=0.45, y=0.155, z=-0.129),
            orientation=overhead_orientation),
        Pose(
            position=Point(x=0.6, y=-0.1, z=-0.129),
            orientation=overhead_orientation)
    ]
    print("Running. Ctrl-c to quit")
    move_to_start(robot, starting_joint_angles)
    idx = 0
    while not rospy.is_shutdown():
        print("\nPicking...")
        pick(robot, block_poses[idx])
        print("\nPlacing...")
        idx = (idx+1) % len(block_poses)
        place(robot, block_poses[idx])
    return 0

if __name__ == '__main__':
    sys.exit(main())
