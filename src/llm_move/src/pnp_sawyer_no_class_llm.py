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
arm = None
arm_pose = None
arm_updated = False

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



## Add the following functions to the llm_pnp_sawyer.py file  
# def enable_robot():
#     robot_state = RobotEnable()
#     robot_state.enable()
# This is no longer needed as the robot is enabled in the main function thoruogh the initialize_robot function


# Callback function for the arm pose subscriber
def arm_callback(data):
    global arm_pose
    global arm_updated

    arm_pose = data
    arm_updated = True


def joint_angles_to_dict(joint_angles):
    return joint_angles


def move_to_neutral():
    """
    Move the arm to a neutral pose.
    """
    print("Moving to neutral pose...")
    arm.move_to_neutral()

    return get_arm_angles()


def move_arm_to_joint_positions(joint_positions):
    global arm_updated

    arm.set_joint_position_speed(0.3)
    arm.move_to_joint_positions(joint_positions)

    while not arm_updated:
        rospy.sleep(0.1)  # Sleep for a short time to avoid busy waiting

    arm_updated = False  # Reset the flag

    joint_angles_dict = joint_angles_to_dict(arm.joint_angles())

    return json.dumps(joint_angles_dict)


def get_arm_angles():
    joint_angles_dict = joint_angles_to_dict(arm.joint_angles())
    return json.dumps(joint_angles_dict)

def run_conversation(user_prompt):
    messages = [
        {
            "role": "system",
            "content": """
                You are a function calling LLM that controls the movement of a Sawyer arm robot to achieve the user prompt.
                
                The Sawyer robot arm has 7 joints that allow it to move in various ways. The joints are as follows:
                - right_j0: Base (Waist) Rotation - This joint allows the entire arm to rotate around the base, enabling the arm to sweep side-to-side.
                - right_j1: Shoulder Out (Shoulder Pitch) - This joint moves the upper arm up and down. Moving the joint in a positive direction lifts the arm up, while a negative direction lowers the arm.
                - right_j2: Shoulder In (Shoulder Yaw) - This joint rotates the upper arm around its own axis, allowing for inward and outward rotation of the arm from the shoulder.
                - right_j3: Elbow - This joint moves the lower arm up and down. A positive direction bends the elbow (lowering the forearm), while a negative direction straightens the elbow (raising the forearm).
                - right_j4: Wrist Up (Wrist Pitch) - This joint moves the wrist up and down. A positive direction bends the wrist towards the palm (lowering the hand), while a negative direction bends the wrist towards the back of the hand (raising the hand).
                - right_j5: Wrist In (Wrist Yaw) - This joint rotates the wrist around its own axis, enabling rotational movement similar to turning a doorknob.
                - right_j6: Wrist Rotation (Wrist Roll) - This joint rotates the end-effector (hand or tool) around its axis, allowing the hand or tool to spin.    
                """,
            "role": "user",
            "content": user_prompt,
        },
    ]
    tools = [
        {
            "type": "function",
            "function": {
                "name": "move_arm_to_joint_positions",
                "description": "Move the Sawyer arm to specific joint positions. Returns the joint angles after the movement.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "joint_positions": {
                            "type": "object",
                            "description": "Dictionary of joint angles in radians to move. Joint names are 'right_j0' to 'right_j6'. Omitting a joint will keep it at its current position.",
                        }
                    },
                    "required": ["joint_positions"],
                },
            },
        },
        {
            "type": "function",
            "function": {
                "name": "get_arm_angles",
                "description": "Returns the current joint angles of the Sawyer arm.",
                "parameters": {},
            },
        },
        {
            "type": "function",
            "function": {
                "name": "move_to_neutral",
                "description": "Move the robot in the base status, resets the robot.",
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
            "move_arm_to_joint_positions": move_arm_to_joint_positions,
            "get_arm_angles": get_arm_angles,
            "move_to_neutral": move_to_neutral,
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

            # specific for move_arm (move_arm_to_joint_positions)
            # Get the parameters of the function
            function_parameters = inspect.signature(function_to_call).parameters

            # Check if function_args contains all the required arguments
            for param in function_parameters:
                if param not in function_args:
                    # If a required argument is missing, add it with a default value
                    current_joint_angles = joint_angles_to_dict(arm.joint_angles())
                    function_args[param] = current_joint_angles[param]

            function_response = function_to_call(**function_args)

            rospy.loginfo(colored(f"Function response: {function_response}", "yellow"))

            messages.append(
                {
                    "tool_call_id": tool_call.id,
                    "role": "tool",
                    "name": function_name,
                    "content": function_response,
                }
            )  # extend conversation with function response
        second_response = client.chat.completions.create(
            model=MODEL, messages=messages
        )  # get a new response from the model where it can see the function response
        response_message = second_response.choices[0].message
        rospy.loginfo(colored(f"Response message: {response_message}", "green"))
        tool_calls = response_message.tool_calls

    return response_message.content


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
        # user_prompt = input("> Enter prompt: ")
        # response = run_conversation(user_prompt)
        # print(colored(f"LLM message: {response}", "cyan"))
        
        print("\nPicking...")
        pick(robot, block_poses[idx])
        print("\nPlacing...")
        idx = (idx+1) % len(block_poses)
        place(robot, block_poses[idx])
    return 0

if __name__ == '__main__':
    sys.exit(main())
