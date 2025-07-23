#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class RecoveryNavigator:
    def __init__(self):
        rospy.init_node('recovery_navigator')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        self.recovery_attempts = 0
        self.max_recovery_attempts = 10
        self.current_pose = None
        self.scan_data = None
        self.position_tolerance = 0.3

        self.plot_centers = [
            [(-0.03, -0.05, 0.0)],              # Home
            [(-1.36, -1.51, 0.0)],              # Leon
            [(-1.46, -0.15, 0.0)],              # Wei Qing
            [(-1.58, 1.11, 0.0)],               # Yong Zun
            [(-0.15, 1.15, math.pi/2)],         # Jia Cheng
            [(1.66, 1.59, -math.pi/2)],         # YT
            [(1.49, 0.06, math.pi / 2)],        # Yong Jie
            [(1.52, -1.16, -math.pi / 2)],      # Gomez
            [(-0.02, -1.25, -math.pi / 2)]      # Kieren
        ]

        rospy.loginfo("Waiting for move_base and services...")
        self.client.wait_for_server()
        self.make_plan.wait_for_service()
        rospy.wait_for_service('/move_base/clear_costmaps')
        rospy.loginfo("Connected to move_base and services.")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        self.scan_data = msg

    def yaw_from_quaternion(self, q):
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def check_goal_reached(self, goal):
        if not self.current_pose:
            return False
        goal_x = goal.target_pose.pose.position.x
        goal_y = goal.target_pose.pose.position.y
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        rospy.loginfo("Distance to goal: %.2f m", distance)
        return distance < self.position_tolerance

    def send_goal(self, goal):
        self.client.send_goal(goal)
        timeout = rospy.Duration(45)
        finished = self.client.wait_for_result(timeout)
        if finished and self.client.get_state() == 3:
            rospy.loginfo("Goal reached successfully!")
            return True
        else:
            self.client.cancel_goal()
            return self.check_goal_reached(goal)

    def can_make_plan(self, goal, start_pose=None):
        req = GetPlanRequest()
        req.start.header.frame_id = "map"
        req.start.header.stamp = rospy.Time.now()
        req.start.pose = start_pose if start_pose else self.current_pose
        req.goal = goal.target_pose
        req.tolerance = 0.5
        try:
            resp = self.make_plan(req)
            return len(resp.plan.poses) > 0
        except:
            return False

    def build_goal(self, x, y, theta):
        q = quaternion_from_euler(0, 0, theta)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = Quaternion(*q)
        return goal

    def has_room_to_rotate(self, min_clearance=0.25, required_clear_count=10):
        if not self.scan_data or not self.scan_data.ranges:
            return False

        ranges = self.scan_data.ranges
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment

        half_fov = math.pi / 2  # ±90 degrees
        start_angle = -half_fov
        end_angle = half_fov

        start_index = int((start_angle - angle_min) / angle_increment)
        end_index = int((end_angle - angle_min) / angle_increment)

        start_index = max(0, start_index)
        end_index = min(len(ranges), end_index)

        clear_count = sum(1 for r in ranges[start_index:end_index] if r > min_clearance)
        rospy.loginfo("Clearance rays in ±90°: %d", clear_count)

        return clear_count >= required_clear_count

    def rotate_until_path_found(self, goal, max_rotation_time=10.0, min_rotation_duration=1.0):
        rospy.loginfo("Rotating left and right to find a valid path...")

        twist = Twist()
        angular_speed = 0.5  # rad/s
        rotate_duration = max_rotation_time / 2.0
        rate = rospy.Rate(10)

        for direction in [1, -1]:  # Try left, then right
            twist.angular.z = direction * angular_speed
            start_time = rospy.Time.now()

            if not self.current_pose:
                continue
            q = self.current_pose.orientation
            _, _, yaw_start = euler_from_quaternion([q.x, q.y, q.z, q.w])

            while (rospy.Time.now() - start_time).to_sec() < rotate_duration and not rospy.is_shutdown():
                self.cmd_pub.publish(twist)
                rospy.sleep(0.1)

                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed >= min_rotation_duration:
                    if self.can_make_plan(goal):
                        if self.current_pose:
                            q = self.current_pose.orientation
                            _, _, yaw_now = euler_from_quaternion([q.x, q.y, q.z, q.w])
                            delta_yaw = abs(yaw_now - yaw_start)
                            if delta_yaw > 0.1:  # about 6 degrees
                                rospy.loginfo("Valid path found after %.1f° rotation!", math.degrees(delta_yaw))
                                self.cmd_pub.publish(Twist())
                                rospy.sleep(0.5)
                                return
                            else:
                                rospy.logwarn("Path found but robot did not rotate enough (%.2f rad)", delta_yaw)

                rate.sleep()

        rospy.logwarn("Rotation did not find a valid path.")
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.5)

    def reverse_slightly(self):
        rospy.loginfo("Reversing slightly to escape tight space...")
        twist = Twist()
        twist.linear.x = -0.05
        for _ in range(5):
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.5)

    def recovery_behavior(self, goal):
        rospy.loginfo("Attempting recovery... (attempt %d)", self.recovery_attempts + 1)

        if self.recovery_attempts == 0:
            try:
                self.clear_costmaps()
                rospy.loginfo("Costmaps cleared.")
            except:
                rospy.logwarn("Costmap clearing failed")

        if self.has_room_to_rotate():
            self.rotate_until_path_found(goal)
        else:
            self.reverse_slightly()

        rospy.sleep(1.0)

    def navigate_to_goal(self, idx, goal):
        rospy.loginfo("Navigating to plot %s", str(idx))
        self.recovery_attempts = 0
        while self.recovery_attempts < self.max_recovery_attempts and not rospy.is_shutdown():
            if self.can_make_plan(goal):
                if self.send_goal(goal):
                    rospy.loginfo("Reached plot %s", str(idx))
                    return True
            self.recovery_behavior(goal)
            self.recovery_attempts += 1
        rospy.logerr("Failed to reach plot %s after recovery", str(idx))
        return False

    def run(self):
        route = rospy.get_param("~options", [5])
        rospy.loginfo("Executing route: %s", route)

        for plot in route:
            if 0 <= plot < len(self.plot_centers):
                goal_pose = self.plot_centers[plot][0]
                goal = self.build_goal(*goal_pose)
                if not self.navigate_to_goal(str(plot), goal):
                    return
                rospy.sleep(1.0)
            else:
                rospy.logwarn("Invalid plot index: %d", plot)

        rospy.loginfo("Navigation route completed.")

if __name__ == '__main__':
    try:
        navigator = RecoveryNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass

