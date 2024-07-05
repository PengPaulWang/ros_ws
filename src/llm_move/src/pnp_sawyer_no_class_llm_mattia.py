#!/usr/bin/env python3

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

# Following are packages from Mattia
import inspect
from intera_interface import Limb, RobotEnable
from intera_core_msgs.msg import EndpointState
import json
from groq import Groq
from termcolor import colored
import os


# Glolab variables - not sure if it is the right place to put them?
# Groq client
client = Groq(
    api_key=os.getenv("GROQ_API_KEY"),
)
MODEL = "llama3-70b-8192"

# Global variables
robot = None


def load_gazebo_models(
    table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),
    table_reference_frame="world",
    block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725)),
    block_reference_frame="world",
):
    model_path = rospkg.RosPack().get_path("sawyer_sim_examples") + "/models/"
    table_xml = ""
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace("\n", "")
    block_xml = ""
    with open(model_path + "block/model.urdf", "r") as block_file:
        block_xml = block_file.read().replace("\n", "")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    try:
        spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        resp_sdf = spawn_sdf(
            "cafe_table", table_xml, "/", table_pose, table_reference_frame
        )
    except rospy.ServiceException as e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    try:
        spawn_urdf = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        resp_urdf = spawn_urdf(
            "block", block_xml, "/", block_pose, block_reference_frame
        )
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    try:
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException as e:
        print("Delete Model service call failed: {0}".format(e))


def initialize_robot(limb="right", hover_distance=0.15, tip_name="right_gripper_tip"):
    global robot

    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()
    robot = {
        'limb_name': limb,
        'tip_name': tip_name,
        'hover_distance': hover_distance,
        'limb': intera_interface.Limb(limb),
        'gripper': intera_interface.Gripper(),
        'rs': rs,
        'init_state': init_state
    }

def move_to_start(start_angles=None):
    global robot
    print("Moving the {0} arm to start pose...".format(robot['limb_name']))
    if not start_angles:
        start_angles = dict(zip(robot['limb'].joint_names(), [0]*7))
    guarded_move_to_joint_position(start_angles)
    gripper_open()

def guarded_move_to_joint_position(joint_angles):
    global robot
    if rospy.is_shutdown():
        return
    if joint_angles:
        robot['limb'].move_to_joint_positions(joint_angles, 5.0)
    else:
        rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

def gripper_open():
    global robot
    robot['gripper'].open()
    rospy.sleep(1.0)

def gripper_close():
    global robot
    robot['gripper'].close()
    rospy.sleep(1.0)

def get_current_pose():
    global robot
    return robot['limb'].endpoint_pose()

# original approach with Pose as argument
def approach(pose):
    global robot
    approach_pose = copy.deepcopy(pose)
    approach_pose.position.z += robot['hover_distance']
    joint_angles = robot['limb'].ik_request(approach_pose, robot['tip_name'])
    robot['limb'].set_joint_position_speed(0.001)
    guarded_move_to_joint_position(joint_angles)
    robot['limb'].set_joint_position_speed(0.1)

# LLM approach with separate arguments to have always values
def approach_pose(position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w):
    global robot
    approach_pose = Pose(
        position = Point(x=position_x, y=position_y, z=position_z),
        orientation = Quaternion(x=orientation_x, y=orientation_y, z=orientation_z, w=orientation_w)
    )
    approach_pose.position.z += robot["hover_distance"]
    joint_angles = robot["limb"].ik_request(approach_pose, robot["tip_name"])
    robot["limb"].set_joint_position_speed(0.001)
    guarded_move_to_joint_position(joint_angles)
    robot["limb"].set_joint_position_speed(0.1)

    return get_current_pose()

def servo_to_pose(pose, time=4.0, steps=400.0):
    global robot
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


def servo_to_pose(
    position_x,
    position_y,
    position_z,
    orientation_x,
    orientation_y,
    orientation_z,
    orientation_w,
    time=4.0,
    steps=400.0,
):
    global robot

    # pose to reach from args
    pose = Pose(
        position=Point(x=position_x, y=position_y, z=position_z),
        orientation=Quaternion(x=orientation_x, y=orientation_y, z=orientation_z, w=orientation_w),
    )

    r = rospy.Rate(1 / (time / steps))
    current_pose = robot["limb"].endpoint_pose()
    ik_delta = Pose()
    ik_delta.position.x = (current_pose["position"].x - pose.position.x) / steps
    ik_delta.position.y = (current_pose["position"].y - pose.position.y) / steps
    ik_delta.position.z = (current_pose["position"].z - pose.position.z) / steps
    ik_delta.orientation.x = (
        current_pose["orientation"].x - pose.orientation.x
    ) / steps
    ik_delta.orientation.y = (
        current_pose["orientation"].y - pose.orientation.y
    ) / steps
    ik_delta.orientation.z = (
        current_pose["orientation"].z - pose.orientation.z
    ) / steps
    ik_delta.orientation.w = (
        current_pose["orientation"].w - pose.orientation.w
    ) / steps
    for d in range(int(steps), -1, -1):
        if rospy.is_shutdown():
            return
        ik_step = Pose()
        ik_step.position.x = d * ik_delta.position.x + pose.position.x
        ik_step.position.y = d * ik_delta.position.y + pose.position.y
        ik_step.position.z = d * ik_delta.position.z + pose.position.z
        ik_step.orientation.x = d * ik_delta.orientation.x + pose.orientation.x
        ik_step.orientation.y = d * ik_delta.orientation.y + pose.orientation.y
        ik_step.orientation.z = d * ik_delta.orientation.z + pose.orientation.z
        ik_step.orientation.w = d * ik_delta.orientation.w + pose.orientation.w
        joint_angles = robot["limb"].ik_request(ik_step, robot["tip_name"])
        if joint_angles:
            robot["limb"].set_joint_positions(joint_angles)
        else:
            rospy.logerr(
                "No Joint Angles provided for move_to_joint_positions. Staying put."
            )
        r.sleep()
    rospy.sleep(1.0)
    return get_current_pose()


# Not used for now, or combination of functions
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
    servo_to_pose(ik_pose)

def pick(pose):
    gripper_open()
    approach(pose)
    servo_to_pose(pose)
    gripper_close()
    retract()

def place(pose):
    approach(pose)
    servo_to_pose(pose)
    gripper_open()
    retract()


def run_conversation(user_prompt):
    messages = [
        {
            "role": "system",
            "content": """
                You are a function calling LLM that controls the movement of a Sawyer arm robot to achieve the user prompt.
                Before moving, check the current position of the arm to calculate the new position.
                """,
            "role": "user",
            "content": user_prompt,
        },
    ]
    tools = [
        {
            "type": "function",
            "function": {
                "name": "approach_pose",
                "description": "This function moves the robot to a specified pose in a single direct move.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "position_x": {
                            "type": "number",
                            "description": "The x-coordinate of the target position.",
                        },
                        "position_y": {
                            "type": "number",
                            "description": "The y-coordinate of the target position.",
                        },
                        "position_z": {
                            "type": "number",
                            "description": "The z-coordinate of the target position.",
                        },
                        "orientation_x": {
                            "type": "number",
                            "description": "The x-component of the target orientation quaternion.",
                        },
                        "orientation_y": {
                            "type": "number",
                            "description": "The y-component of the target orientation quaternion.",
                        },
                        "orientation_z": {
                            "type": "number",
                            "description": "The z-component of the target orientation quaternion.",
                        },
                        "orientation_w": {
                            "type": "number",
                            "description": "The w-component of the target orientation quaternion.",
                        },
                    },
                    "required": [
                        "position_x",
                        "position_y",
                        "position_z",
                        "orientation_x",
                        "orientation_y",
                        "orientation_z",
                        "orientation_w",
                    ],
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "servo_to_pose",
                "description": "This function moves the robot arm to a specified pose in multiple small gradual steps. ",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "position_x": {
                            "type": "number",
                            "description": "The x-coordinate of the target position.",
                        },
                        "position_y": {
                            "type": "number",
                            "description": "The y-coordinate of the target position.",
                        },
                        "position_z": {
                            "type": "number",
                            "description": "The z-coordinate of the target position.",
                        },
                        "orientation_x": {
                            "type": "number",
                            "description": "The x-component of the target orientation quaternion.",
                        },
                        "orientation_y": {
                            "type": "number",
                            "description": "The y-component of the target orientation quaternion.",
                        },
                        "orientation_z": {
                            "type": "number",
                            "description": "The z-component of the target orientation quaternion.",
                        },
                        "orientation_w": {
                            "type": "number",
                            "description": "The w-component of the target orientation quaternion.",
                        },
                    },
                    "required": [
                        "position_x",
                        "position_y",
                        "position_z",
                        "orientation_x",
                        "orientation_y",
                        "orientation_z",
                        "orientation_w",
                    ],
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "get_current_pose",
                "description": "This function returns the current pose of the robot's arm",
                "parameters": {},
            },
        },
    ]

    response = client.chat.completions.create(
        model=MODEL, messages=messages, tools=tools, tool_choice="auto", max_tokens=4096
    )

    response_message = response.choices[0].message
    tool_calls = response_message.tool_calls

    while tool_calls:
        available_functions = {
            "approach_pose": approach_pose,
            "servo_to_pose": servo_to_pose,
            "get_current_pose": get_current_pose,
        }

        rospy.loginfo(colored(f"Response message: {response_message}", "green"))
        messages.append(response_message)  # extend conversation with assistant's reply

        for tool_call in tool_calls:
            function_name = tool_call.function.name
            function_to_call = available_functions[function_name]
            function_args = json.loads(tool_call.function.arguments) if tool_call.function.arguments else {}
            rospy.loginfo(
                colored(
                    f"Calling function {function_name} with arguments {function_args}",
                    "yellow",
                )
            )

            function_response = function_to_call(**function_args)

            rospy.loginfo(colored(f"Function response: {function_response}", "yellow"))

            messages.append(
                {
                    "tool_call_id": tool_call.id,
                    "role": "tool",
                    "name": function_name,
                    "content": str(function_response), # force to string, pose returns pose object
                }
            )  # extend conversation with function response
        second_response = client.chat.completions.create(
            model=MODEL, messages=messages
        )  # get a new response from the model where it can see the function response
        response_message = second_response.choices[0].message
        rospy.loginfo(colored(f"Response message: {response_message}", "green"))
        tool_calls = response_message.tool_calls

    return response_message.content

def main():
    rospy.init_node("pnp_sawyer_no_class_llm_mattia")
    load_gazebo_models()
    rospy.on_shutdown(delete_gazebo_models)

    limb = 'right'
    hover_distance = 0.15

    print("Initializing robot...")
    initialize_robot(limb, hover_distance)
    print("Pose: ", get_current_pose())

    starting_joint_angles = {'right_j0': -0.041662954890248294,
                             'right_j1': -1.0258291091425074,
                             'right_j2': 0.0293680414401436,
                             'right_j3': 2.17518162913313,
                             'right_j4': -0.06703022873354225,
                             'right_j5': 0.3968371433926965,
                             'right_j6': 1.7659649178699421}
    
    print("Moving to start...")
    move_to_start(starting_joint_angles)
    print("Pose: ", get_current_pose())

    # Loop conversation
    while not rospy.is_shutdown():
        user_prompt = input("> Enter prompt: ")
        response = run_conversation(user_prompt)
        print(colored(f"LLM message: {response}", "cyan"))

        #print("\nPicking...")
        #pick(robot, block_poses[idx])
        #print("\nPlacing...")
        #idx = (idx+1) % len(block_poses)
        #place(robot, block_poses[idx])


if __name__ == '__main__':
    main()