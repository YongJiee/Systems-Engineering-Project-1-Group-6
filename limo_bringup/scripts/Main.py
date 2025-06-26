#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import math

# Define coordinates (x, y in meters)
locations = {
    "Home": (0.68, -0.01),
    "Point A": (2, 0.0)  # Replace with your real world Point A coordinates
}

def send_goal(client, x, y, yaw_rad=0.0):
    quat = quaternion_from_euler(0, 0, yaw_rad)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation = Quaternion(*quat)

    rospy.loginfo("Sending goal: x={:.2f}, y={:.2f}, yaw={:.1f}°".format(x, y, math.degrees(yaw_rad)))
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Goal reached.\n")

def main():
    rospy.init_node('home_to_a_and_back')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base.\n")

    # Go to Point A
    send_goal(client, *locations["Point A"], yaw_rad=0.0)

    # Return to Home
    send_goal(client, *locations["Home"], yaw_rad=math.pi)  # face opposite direction on return

    rospy.loginfo("Returned to Home. Mission complete.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

