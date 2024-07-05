#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 

import os
#from dotenv import load_dotenv # doesnt work from launch as rel path
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
odom = None
odom_updated = False
cmd_vel_pub = None
odom_sub = None


# Callback function for the pose subscriber
def odom_callback(data):
    global odom
    global odom_updated

    odom = data
    odom_updated = True

def pose_to_dict(pose):
    return {
        "position": {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
        },
        "orientation": {
            "x": pose.orientation.x,
            "y": pose.orientation.y,
            "z": pose.orientation.z,
            "w": pose.orientation.w,
        },
    }

# Helper function to send cmd_vel with duration
# After duration seconds, stop the robot as default
def send_cmd_vel(linear_x, linear_y, linear_z, 
                 angular_x, angular_y, angular_z, 
                 duration, stop_robot=True):

    global odom_updated

    # Create Twist message
    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = linear_y
    twist.linear.z = linear_z
    twist.angular.x = angular_x
    twist.angular.y = angular_y
    twist.angular.z = angular_z

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
    while not odom_updated:
        rospy.sleep(0.1)  # Sleep for a short time to avoid busy waiting

    odom_updated = False  # Reset the flag

    # Convert the dictionary to a JSON string and return it
    pose_dict = pose_to_dict(odom.pose.pose)

    return json.dumps(pose_dict)


def run_conversation(user_prompt):
    # Step 1: send the conversation and available functions to the model
    messages = [
        # try to system prompt
        #{
        #    "role": "system",
        #    "content": "You are a function calling LLM that controls the movement of a robot using the send_cmd_vel function to achieve the user prompt. The function returs the Pose of the robot after the movement.",
        #},
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
                "description": "Publish cmd_vel message to control the robot movement. Returns an Odometry message after the movement.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "linear_x": {
                            "type": "number",
                            "description": "The linear velocity along the x-axis",
                        },
                        "linear_y": {
                            "type": "number",
                            "description": "The linear velocity along the y-axis",
                        },
                        "linear_z": {
                            "type": "number",
                            "description": "The linear velocity along the z-axis",
                        },
                        "angular_x": {
                            "type": "number",
                            "description": "The angular velocity around the x-axis",
                        },
                        "angular_y": {
                            "type": "number",
                            "description": "The angular velocity around the y-axis",
                        },
                        "angular_z": {
                            "type": "number",
                            "description": "The angular velocity around the z-axis",
                        },
                        "duration": {
                            "type": "number",
                            "description": "The duration of the movement in seconds",
                        },
                    },
                    "required": [
                        "linear_x", "linear_y", "linear_z",
                        "angular_x", "angular_y", "angular_z",
                        "duration"
                    ],
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

            # rosllm use **kwargs? idk can find better ways to handle json description and real function call - reflection/pydantic?
            function_response = function_to_call(
                linear_x=function_args["linear_x"],
                linear_y=function_args["linear_y"],
                linear_z=function_args["linear_z"],
                angular_x=function_args["angular_x"],
                angular_y=function_args["angular_y"],
                angular_z=function_args["angular_z"],
                duration=function_args["duration"],
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
    global cmd_vel_pub, odom_sub

    rospy.init_node("robot_llm")

    #cmd_vel_topic = rospy.get_param("~cmd_vel")
    #odom_topic = rospy.get_param("~odom")

    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)

    # Wait for the first pose to be updated
    while odom is None:
        rospy.sleep(0.1)  # Sleep for a short time to avoid busy waiting

    pos = odom.pose.pose.position
    print("Initial pose: x={}, y={}, theta={}".format(pos.x, pos.y, pos.z))

    # Run the conversation loop
    while not rospy.is_shutdown():
        user_prompt = input("> Enter prompt: ")
        response = run_conversation(user_prompt)
        print(colored(f"LLM message: {response}", "cyan"))

    # rospy.spin()


if __name__ == "__main__":
    main()
