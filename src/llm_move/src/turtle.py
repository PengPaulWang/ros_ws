#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import os
from groq import Groq
import json

# debugging - pretty print
from pprint import pprint
from termcolor import colored


# Groq client
client = Groq(
    api_key=os.getenv("GROQ_API_KEY"),
)
MODEL = "llama3-70b-8192"

# Global variables
pose = None
pose_updated = False
cmd_vel_pub = None
pose_sub = None


# Callback function for the pose subscriber
def pose_callback(data):
    global pose
    global pose_updated

    pose = data
    pose_updated = True


"""
The turtlesim node in ROS simulates a differential drive robot, 
which can only move in the forward/backward direction (linear.x) and rotate around its center (angular.z). 

It cannot move sideways (linear.y), nor can it rotate around the x or y axes (angular.x, angular.y).

This is a simplification often used for wheeled robots and mobile robots in 2D environments. 
If you're working with a robot that can move sideways, such as a drone or a robot with mecanum wheels, 
you would need a different simulator that can handle these types of movements.
"""

# Helper function to send cmd_vel with duration
# After duration seconds, stop the robot as default
def send_cmd_vel(linear_velocity, angular_velocity, duration, stop_robot=True):
    global pose_updated

    # Create Twist message
    twist = Twist()
    twist.linear.x = linear_velocity
    twist.angular.z = angular_velocity

    # Get the current time
    start_time = rospy.Time.now().secs

    # Continue to publish the Twist message until the duration has elapsed
    while rospy.Time.now().secs - start_time < duration:
        cmd_vel_pub.publish(twist)

    # Stop the robot if stop_robot is True
    if stop_robot:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)

    # Wait for the pose to be updated
    while not pose_updated:
        rospy.sleep(0.1)  # Sleep for a short time to avoid busy waiting

    pose_updated = False  # Reset the flag

    # Convert the Pose object to a dictionary
    pose_dict = {
        "x": pose.x,
        "y": pose.y,
        "theta": pose.theta,
    }

    # Convert the dictionary to a JSON string and return it
    return json.dumps(pose_dict)


def run_conversation(user_prompt):
    # Step 1: send the conversation and available functions to the model
    messages = [
        {
            "role": "system",
            "content": "You are a function calling LLM that controls the movement of a robot using the send_cmd_vel function to achieve the user prompt. The function returs the Pose of the robot after the movement.",
        },
        {
            "role": "user",
            "content": user_prompt,
        },
    ]
    tools = [
        {
            "type": "function",
            "function": {
                "name": "send_cmd_vel",
                "description": "Send a Twist message to move the robot and return the Pose of the robot after movement",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "linear_velocity": {
                            "type": "number",
                            "description": "The linear velocity along the x-axis",
                        },
                        "angular_velocity": {
                            "type": "number",
                            "description": "The angular velocity around the z-axis",
                        },
                        "duration": {
                            "type": "number",
                            "description": "The duration of the movement in seconds",
                        },
                    },
                    "required": ["linear_velocity", "angular_velocity", "duration"],
                },
            },
        }
    ]

    # Send the conversation to the model
    response = client.chat.completions.create(
        model=MODEL, 
        messages=messages, 
        tools=tools, 
        tool_choice="auto", 
        max_tokens=4096
    )

    response_message = response.choices[0].message
    tool_calls = response_message.tool_calls

    # Step 2: check if the model wanted to call a function -> repeat until no more tool calls
    while tool_calls:

        # Step 3: call the function
        # Note: the JSON response may not always be valid; be sure to handle errors
        # TODO: see if function is available?
        available_functions = {
            "send_cmd_vel": send_cmd_vel,
        }  # only one function in this example, but you can have multiple

        rospy.loginfo(
            colored(f"Response message: {response_message}", "green")
        )
        messages.append(response_message)  # extend conversation with assistant's reply

        # Step 4: send the info for each function call and function response to the model
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
                linear_velocity=function_args.get("linear_velocity"),
                angular_velocity=function_args.get("angular_velocity"),
                duration=function_args.get("duration"),
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
            model=MODEL, 
            messages=messages
        )  # get a new response from the model where it can see the function response
        response_message = second_response.choices[0].message
        rospy.loginfo(
            colored(f"Response message: {response_message}", "green")
        )
        tool_calls = response_message.tool_calls

    return response_message.content


def main():
    global cmd_vel_pub, pose_sub

    rospy.init_node("robot_llm")

    cmd_vel_topic = rospy.get_param("~cmd_vel")
    pose_topic = rospy.get_param("~pose")

    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
    pose_sub = rospy.Subscriber(pose_topic, Pose, pose_callback)

    # Wait for the first pose to be updated
    while pose is None:
        rospy.sleep(0.1)  # Sleep for a short time to avoid busy waiting

    print("Initial pose: x={}, y={}, theta={}".format(pose.x, pose.y, pose.theta))

    # Run the conversation loop
    while not rospy.is_shutdown():
        user_prompt = input("> Enter prompt: ")
        response = run_conversation(user_prompt)
        print(colored(f"LLM message: {response}", "cyan"))

    # rospy.spin()


if __name__ == "__main__":
    main()
