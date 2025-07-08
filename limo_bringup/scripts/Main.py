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
<<<<<<< HEAD
        self.recovery_attempts = 0
        rospy.init_node('smart_navigation_node')

        self.plot_centers = [
            (0.0, 0.0, 0.0),                # 0 - Home
            (1.0, 1.0, 0.0),                # 1
            (2.0, 1.0, math.pi / 2),        # 2
            (3.0, 1.0, math.pi),            # 3
            (1.0, 2.0, -math.pi / 2),       # 4
            (3.0, 2.0, 0.0),                # 5
            (1.2, 0.15, math.pi / 2),        # 6
            (2.0, 3.0, -math.pi / 4),       # 7
            (3.0, 3.0, math.pi)             # 8
        ]
=======
        rospy.init_node('recovery_navigator')
>>>>>>> parent of bf3a69d... Navigation with lidar working and fine tune revovery

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_pose = None

        rospy.loginfo("Waiting for move_base server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

<<<<<<< HEAD
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

        # Use forward hemisphere: 60° left to 60° right
        num_points = len(self.scan_data)
        angle_range_deg = 120  # total 120° FOV
        step_deg = 5
        center_index = num_points // 2
        angle_span = int(angle_range_deg / step_deg / 2)

        valid_indices = range(center_index - angle_span, center_index + angle_span)
        valid_ranges = [self.scan_data[i] for i in valid_indices if 0 <= i < num_points]

        for r in valid_ranges:
            if 0 < r < 0.6:
                return False

        return True

    def recover_smart(self):
        rospy.loginfo("Smart recovery: clear costmap, rotate until plan found, or reverse if blocked.")
        if self.client.get_state() not in [3, 4, 5, 9]:  # Avoid cancelling completed goals
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
            while not rospy.is_shutdown():
                self.cmd_vel_pub.publish(twist)
                if self.can_make_plan():
                    break
                rate.sleep()
            self.cmd_vel_pub.publish(Twist())
            rospy.loginfo("Valid plan detected. Stopping rotation.")
            return  # Exit after successful rotation and plan

        else:
            rospy.logwarn("Not enough space. Reversing slightly... (attempt #%d)", self.recovery_attempts + 1)
            twist = Twist()
            twist.linear.x = -0.2
            reverse_steps = max(3, 8 - self.recovery_attempts * 2)
            for _ in range(reverse_steps):
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.1)
            self.cmd_vel_pub.publish(Twist())
            self.recovery_attempts += 1
            rospy.sleep(1.0)
            return  # Exit after reversing to avoid repeating within one call
            self.recovery_attempts += 1
        rospy.sleep(1.0)

        if self.has_room_to_rotate():
            rospy.loginfo("Enough space to rotate. Rotating until valid plan is found...")
            twist = Twist()
            twist.angular.z = 0.4
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                self.cmd_vel_pub.publish(twist)
                if self.can_make_plan():
                    break
                rate.sleep()
            self.cmd_vel_pub.publish(Twist())
            rospy.loginfo("Valid plan detected. Stopping rotation.")
            return  # Exit after successful rotation and plan

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
=======
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

>>>>>>> parent of bf3a69d... Navigation with lidar working and fine tune revovery
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

<<<<<<< HEAD
        for i, goal in enumerate(self.goal_list):
            self.recovery_attempts = 0
            self.current_goal = goal
            rospy.loginfo("Navigating to plot %d", rospy.get_param("~options", [])[i])

            start_time = rospy.Time.now()
            while not self.can_make_plan():
                if (rospy.Time.now() - start_time).to_sec() < 5.0:
                    rospy.loginfo("Waiting for planner to succeed...")
                    rospy.sleep(1.0)
                else:
                    rospy.logwarn("No path found after 5 seconds. Trying recovery...")
                    self.recover_smart()
                    start_time = rospy.Time.now()
=======
        self.cmd_pub.publish(Twist())  # stop robot

    def send_goal_with_recovery(self, x, y, yaw_rad=0.0):
        quat = quaternion_from_euler(0, 0, yaw_rad)
>>>>>>> parent of bf3a69d... Navigation with lidar working and fine tune revovery

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

