#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from tf.transformations import quaternion_from_euler

class RecoveryNavigator:
    def __init__(self):
        rospy.init_node('recovery_navigator')

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_pose = None

        rospy.loginfo("Waiting for move_base server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def can_plan_path(self, goal_pose):
        if self.current_pose is None:
            rospy.logwarn("Current pose unknown. Cannot plan.")
            return False

        try:
            rospy.wait_for_service('/move_base/make_plan', timeout=2.0)
            make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

            start = PoseStamped()
            start.header.frame_id = "map"
            start.header.stamp = rospy.Time.now()
            start.pose.position = self.current_pose.position
            # Safe default orientation
            start.pose.orientation.w = 1.0

            goal_pose.header.stamp = rospy.Time.now()

            resp = make_plan(start=start, goal=goal_pose, tolerance=0.1)
            if not resp.plan.poses:
                rospy.logwarn("make_plan returned empty path.")
                return False

            rospy.loginfo("make_plan returned valid path ({} poses).".format(len(resp.plan.poses)))
            return True

        except rospy.ServiceException as e:
            rospy.logerr("make_plan service call failed: %s", e)
            return False
        except rospy.ROSException as e:
            rospy.logerr("Timeout waiting for make_plan: %s", e)
            return False

    def recover_by_rotation_until_path_exists(self, goal_pose, timeout_sec=15):
        rospy.logwarn("Starting recovery: rotate until path can be planned...")
        twist = Twist()
        twist.angular.z = 0.4

        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if rospy.Time.now() - start_time > rospy.Duration(timeout_sec):
                rospy.logerr("Recovery timeout. Still no valid path.")
                break

            if self.can_plan_path(goal_pose):
                rospy.loginfo("Path to goal is now available. Stopping recovery.")
                break

            self.cmd_pub.publish(twist)
            rate.sleep()

        self.cmd_pub.publish(Twist())  # stop robot

    def send_goal_with_recovery(self, x, y, yaw_rad=0.0):
        quat = quaternion_from_euler(0, 0, yaw_rad)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = Quaternion(*quat)

        goal_pose = PoseStamped()
        goal_pose.header = goal.target_pose.header
        goal_pose.pose = goal.target_pose.pose

        rospy.loginfo("Sending goal to x={:.2f}, y={:.2f}".format(x, y))
        self.client.send_goal(goal)
        success = self.client.wait_for_result(rospy.Duration(30))

        if not success or self.client.get_state() != 3:
            rospy.logwarn("Goal failed. Attempting recovery...")
            self.recover_by_rotation_until_path_exists(goal_pose)
            rospy.sleep(1.0)
            rospy.loginfo("Retrying goal...")
            self.client.send_goal(goal)
            self.client.wait_for_result()

        if self.client.get_state() == 3:
            rospy.loginfo("Goal succeeded.\n")
        else:
            rospy.logerr("Goal retry failed.\n")

    def run(self):
        # Example goals
        goals = [
            (1.48, 0.48, 0.0),            # Point A
            (0.68, -0.01, math.pi)        # Home
        ]
        for x, y, yaw in goals:
            self.send_goal_with_recovery(x, y, yaw)

if __name__ == '__main__':
    try:
        node = RecoveryNavigator()
        node.run()
    except rospy.ROSInterruptException:
        pass

