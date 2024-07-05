#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

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


# Helper function to send cmd_vel with duration
# After duration seconds, stop the robot as default
def send_cmd_vel(twist, duration, stop_robot=True):
    global pose_updated

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

    return pose  # Return the updated pose


def main():
    global cmd_vel_pub, pose_sub

    rospy.init_node("robot_llm")

    cmd_vel_topic = rospy.get_param("~cmd_vel")
    pose_topic = rospy.get_param("~pose")

    pose_sub = rospy.Subscriber(pose_topic, Pose, pose_callback)

    # Wait for the first pose to be updated
    while pose is None:
        rospy.sleep(0.1)  # Sleep for a short time to avoid busy waiting

    print("Initial pose: x={}, y={}, theta={}".format(pose.x, pose.y, pose.theta))

    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

    twist = Twist()
    twist.linear.x = 1.0
    twist.angular.z = 0.0

    # Get the pose after the robot moves and stops
    pose_after_move = send_cmd_vel(twist, 2)

    print(
        "Pose after move: x={}, y={}, theta={}".format(
            pose_after_move.x, pose_after_move.y, pose_after_move.theta
        )
    )

    rospy.spin()


if __name__ == "__main__":
    main()
