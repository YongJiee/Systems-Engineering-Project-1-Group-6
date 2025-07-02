#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty

class SmartNavigator:
    def __init__(self):
        self.recovery_attempts = 0
        rospy.init_node('smart_navigation_node')

        self.plot_centers = [
            (0.0, 0.0, 0.0),                # 0 - Home
            (1.0, 1.0, 0.0),                # 1
            (2.0, 1.0, math.pi / 2),        # 2
            (3.0, 1.0, math.pi),            # 3
            (1.0, 2.0, -math.pi / 2),       # 4
            (3.0, 2.0, 0.0),                # 5
            (1.0, 3.0, math.pi / 4),        # 6
            (2.0, 3.0, -math.pi / 4),       # 7
            (3.0, 3.0, math.pi)             # 8
        ]

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

        self.make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.make_plan.wait_for_service()
        rospy.loginfo("Connected to /move_base/make_plan service.")

        self.goal_list = []
        self.current_goal = None

        self.scan_data = []
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        self.scan_data = msg.ranges

    def get_goals_from_params(self):
        raw_indices = rospy.get_param("~options", [1, 2, 3])
        self.goal_list = []

        for one_based_idx in raw_indices:
            idx = one_based_idx  # Plot 0 is now valid
            if not (0 <= idx < len(self.plot_centers)):
                rospy.logwarn("Invalid plot index: %d (skipping)", one_based_idx)
                continue
            x, y, theta = self.plot_centers[idx]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            qx, qy, qz, qw = quaternion_from_euler(0, 0, theta)
            goal.target_pose.pose.orientation.x = qx
            goal.target_pose.pose.orientation.y = qy
            goal.target_pose.pose.orientation.z = qz
            goal.target_pose.pose.orientation.w = qw
            self.goal_list.append(goal)
            rospy.loginfo("Queued plot #%d: x=%.2f, y=%.2f, θ=%.2f rad", one_based_idx, x, y, theta)

    def send_goal(self):
        rospy.loginfo("Sending goal to move_base...")
        self.client.send_goal(self.current_goal)
        self.client.wait_for_result(rospy.Duration(30))
        return self.client.get_state() == 3

    def has_room_to_rotate(self):
        if not self.scan_data:
            rospy.logwarn("No LIDAR data available.")
            return False
        valid_ranges = [r for i, r in enumerate(self.scan_data) if i % 10 == 0]
        return all(r > 0.6 or r == float('inf') for r in valid_ranges)

    def recover_smart(self):
        rospy.loginfo("Smart recovery: clear costmap, rotate until plan found, or reverse if blocked.")
        self.client.cancel_goal()

        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=2.0)
            clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_costmap()
            rospy.loginfo("Cleared costmaps.")
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to clear costmaps: %s", e)

        rospy.sleep(1.0)

        if self.has_room_to_rotate():
            rospy.loginfo("Enough space to rotate. Rotating until valid plan is found...")
            twist = Twist()
            twist.angular.z = 0.4
            rate = rospy.Rate(10)
            while not rospy.is_shutdown() and not self.can_make_plan():
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            self.cmd_vel_pub.publish(Twist())
            rospy.loginfo("Valid plan detected. Stopping rotation.")
        else:
            rospy.logwarn("Not enough space. Reversing... (attempt #%d)", self.recovery_attempts + 1)
            twist = Twist()
            twist.linear.x = -0.2
            # reduce reversing distance gradually if repeating
            reverse_steps = max(3, 8 - self.recovery_attempts * 2)
            for _ in range(reverse_steps):
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.1)
            self.cmd_vel_pub.publish(Twist())
            self.recovery_attempts += 1
        rospy.sleep(1.0)

        if self.has_room_to_rotate():
            rospy.loginfo("Enough space to rotate. Rotating until valid plan is found...")
            twist = Twist()
            twist.angular.z = 0.4
            rate = rospy.Rate(10)
            while not rospy.is_shutdown() and not self.can_make_plan():
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            self.cmd_vel_pub.publish(Twist())
            rospy.loginfo("Valid plan detected. Stopping rotation.")
        else:
            rospy.logwarn("Not enough space. Reversing...")
            twist = Twist()
            twist.linear.x = -0.2
            for _ in range(8):  # reduce reverse steps to ~0.8 seconds
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.1)
            self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1.0)

    def can_make_plan(self):
        req = GetPlanRequest()
        req.start.header.frame_id = "map"
        req.start.header.stamp = rospy.Time.now()
        req.start.pose = self.current_goal.target_pose.pose
        req.goal = self.current_goal.target_pose
        req.tolerance = 0.5
        try:
            resp = self.make_plan(req)
            return len(resp.plan.poses) > 0
        except rospy.ServiceException as e:
            rospy.logerr("make_plan failed: %s", e)
            return False

    def run(self):
        self.get_goals_from_params()

        if len(self.goal_list) == 0:
            rospy.logwarn("No goals provided. Exiting.")
            return

        for i, goal in enumerate(self.goal_list):
            self.recovery_attempts = 0
            self.current_goal = goal
            rospy.loginfo("Navigating to plot %d of %d", i + 1, len(self.goal_list))

            while not self.can_make_plan():
                rospy.logwarn("No path found. Trying recovery...")
                self.recover_smart()

            while not self.send_goal():
                rospy.logwarn("Goal failed. Trying recovery...")
                self.recover_smart()
                while not self.can_make_plan():
                    rospy.logwarn("Still no path. Trying recovery again...")
                    self.recover_smart()

        rospy.loginfo("Finished visiting all plots.")

if __name__ == '__main__':
    try:
        nav = SmartNavigator()
        nav.run()
    except rospy.ROSInterruptException:
        pass

