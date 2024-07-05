#!/usr/bin/env python3

import rospy
import intera_interface
from intera_interface import Limb, RobotEnable
from intera_core_msgs.msg import EndpointState
import json
from groq import Groq
from termcolor import colored
import os

# Groq client
client = Groq(
    api_key=os.getenv("GROQ_API_KEY"),
)
MODEL = "llama3-70b-8192"

# groq API Key: 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

# export GROQ_API_KEY="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"

# sudo apt-get install gazebo9 ros-melodic-qt-build ros-melodic-gazebo-ros-control ros-melodic-gazebo-ros-pkgs ros-melodic-ros-control ros-melodic-control-toolbox ros-melodic-realtime-tools ros-melodic-ros-controllers ros-melodic-xacro python-wstool ros-melodic-tf-conversions ros-melodic-kdl-parser


# Global variables
arm = None
arm_pose = None
arm_updated = False


def enable_robot():
    robot_state = RobotEnable()
    robot_state.enable()


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


def interpolate_movement(start_movement, end_movement):
    """
    Interpolate between two movements to create a smooth motion.

    :param start_movement: Starting joint angles for the movement
    :param end_movement: Ending joint angles for the movement
    :return: List of joint angles for the smooth motion
    """
    smooth_motion = []
    steps = 50  # Number of interpolation steps
    for i in range(steps):
        alpha = float(i) / (steps - 1)  # Interpolation factor
        smooth_step = {}
        for joint, start_angle in start_movement.items():
            end_angle = end_movement[joint]
            interpolated_angle = start_angle + alpha * (end_angle - start_angle)
            smooth_step[joint] = interpolated_angle
        smooth_motion.append(smooth_step)
    return smooth_motion


def move_arm_to_joint_positions(joint_positions):
    global arm_updated

    arm.set_joint_position_speed(0.3)
    arm.move_to_joint_positions(joint_positions)

    while not arm_updated:
        rospy.sleep(0.1)  # Sleep for a short time to avoid busy waiting

    arm_updated = False  # Reset the flag

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
                - right_j1: Shoulder Out (Shoulder Pitch) - This joint moves the upper arm up and down, similar to lifting your arm up and down from the shoulder.
                - right_j2: Shoulder In (Shoulder Yaw) - This joint rotates the upper arm around its own axis, allowing for inward and outward rotation of the arm from the shoulder.
                - right_j3: Elbow - This joint moves the lower arm up and down, similar to bending and straightening your elbow.
                - right_j4: Wrist Up (Wrist Pitch) - This joint moves the wrist up and down, akin to bending your wrist towards the palm or the back of your hand.
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
    ]

    response = client.chat.completions.create(
        model=MODEL, messages=messages, tools=tools, tool_choice="auto", max_tokens=4096
    )

    response_message = response.choices[0].message
    tool_calls = response_message.tool_calls

    while tool_calls:
        available_functions = {
            "move_arm_to_joint_positions": move_arm_to_joint_positions,
        }

        rospy.loginfo(colored(f"Response message: {response_message}", "green"))
        messages.append(response_message)  # extend conversation with assistant's reply

        for tool_call in tool_calls:
            function_name = tool_call.function.name
            function_to_call = available_functions[function_name]
            function_args = json.loads(tool_call.function.arguments)
            rospy.loginfo(
                colored(
                    f"Calling function {function_name} with arguments {function_args}",
                    "yellow",
                )
            )

            function_response = function_to_call(
                joint_positions=function_args["joint_positions"]
            )

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


def main():
    global arm

    rospy.init_node("robot_llm")

    enable_robot()
    arm = Limb("right")

    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, arm_callback)

    while arm_pose is None:
        rospy.sleep(0.1)  # Sleep for a short time to avoid busy waiting

    joint_angles = arm.joint_angles()
    print("Initial joint angles: {}".format(joint_angles))

    while not rospy.is_shutdown():
        user_prompt = input("> Enter prompt: ")
        response = run_conversation(user_prompt)
        print(colored(f"LLM message: {response}", "cyan"))

    # rospy.spin()


if __name__ == "__main__":
    main()
