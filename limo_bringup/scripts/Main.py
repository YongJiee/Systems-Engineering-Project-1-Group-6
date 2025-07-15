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
        self.position_tolerance = 0.3  # meters

        self.plot_centers = [
            [(0.0, 0.0, 0.0)],
            [(0.5, 0.0, 0.0)],
            [(2.5, 0.2, math.pi / 2)],
            [(1.7, 0.5, 3.1)],
            [(0.0, 1.35, math.pi / 2)],
            [  # Plot 5 with two entrances and one goal
                (1.8, 1.04, -math.pi/2),  # Entrance A
                (0.91, 1.68, 0.0),        # Entrance B
                (1.68, 1.71, -math.pi/2)  # Goal
            ],
            [(1.55, 0.0, math.pi / 2)],
            [(1.49, -1.21, -math.pi / 2)],
            [(0.0, -1.25, -math.pi / 2)]
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

    def has_room_to_rotate(self, min_clearance=0.25):
        if not self.scan_data or not self.scan_data.ranges:
            rospy.logwarn("No scan data available for rotation check.")
            return False
        valid_ranges = [r for r in self.scan_data.ranges if not math.isinf(r) and not math.isnan(r) and r > 0]
        if not valid_ranges:
            rospy.logwarn("All scan readings are invalid.")
            return False
        min_distance = min(valid_ranges)
        rospy.loginfo("Min clearance: %.2f m, Required: %.2f m", min_distance, min_clearance)
        return min_distance > min_clearance

    def rotate_until_path_found(self, goal, max_rotation_time=10.0):
        rospy.loginfo("Rotating to find valid path...")
        twist = Twist()
        twist.angular.z = 0.3
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > max_rotation_time:
                rospy.logwarn("Rotation timeout reached.")
                break
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
            if self.can_make_plan(goal):
                rospy.loginfo("Valid path found during rotation!")
                break
            rate.sleep()
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
        try:
            self.clear_costmaps()
        except:
            rospy.logwarn("Costmap clearing failed")

        if self.has_room_to_rotate():
            self.rotate_until_path_found(goal)
        else:
            self.reverse_slightly()
        rospy.sleep(1.0)

    def choose_best_entrance_to_plot5_goal(self):
        entrance_indices = [0, 1]
        goal_pose = self.plot_centers[5][2]
        goal = self.build_goal(*goal_pose)

        best_idx = None
        best_total_length = float('inf')

        for i in entrance_indices:
            x, y, theta = self.plot_centers[5][i]
            entrance_goal = self.build_goal(x, y, theta)

            req1 = GetPlanRequest()
            req1.start.header.frame_id = "map"
            req1.start.header.stamp = rospy.Time.now()
            req1.start.pose = self.current_pose
            req1.goal = entrance_goal.target_pose
            req1.tolerance = 0.5

            req2 = GetPlanRequest()
            req2.start.header.frame_id = "map"
            req2.start.header.stamp = rospy.Time.now()
            req2.start.pose = entrance_goal.target_pose.pose
            req2.goal = goal.target_pose
            req2.tolerance = 0.5

            try:
                resp1 = self.make_plan(req1)
                resp2 = self.make_plan(req2)

                len1 = sum(math.hypot(
                    resp1.plan.poses[j+1].pose.position.x - resp1.plan.poses[j].pose.position.x,
                    resp1.plan.poses[j+1].pose.position.y - resp1.plan.poses[j].pose.position.y
                ) for j in range(len(resp1.plan.poses)-1))

                len2 = sum(math.hypot(
                    resp2.plan.poses[j+1].pose.position.x - resp2.plan.poses[j].pose.position.x,
                    resp2.plan.poses[j+1].pose.position.y - resp2.plan.poses[j].pose.position.y
                ) for j in range(len(resp2.plan.poses)-1))

                total_len = len1 + len2
                rospy.loginfo("Entrance %d: to entrance = %.2f, to goal = %.2f, total = %.2f", i, len1, len2, total_len)

                if total_len < best_total_length:
                    best_total_length = total_len
                    best_idx = i
            except Exception as e:
                rospy.logwarn("Failed to evaluate entrance %d: %s", i, str(e))

        return best_idx, goal

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

        for i, plot in enumerate(route):
            if plot == 5:
                entrance_idx, final_goal = self.choose_best_entrance_to_plot5_goal()
                if entrance_idx is None:
                    rospy.logerr("No valid path to plot 5 goal.")
                    return
                entrance_pose = self.plot_centers[5][entrance_idx]
                entrance_goal = self.build_goal(*entrance_pose)

                if not self.navigate_to_goal("5 entrance %d" % entrance_idx, entrance_goal):
                    return
                rospy.sleep(1.0)

                if not self.navigate_to_goal("5 goal", final_goal):
                    return
                rospy.sleep(1.0)

                # Look ahead for next plot to determine best exit
                next_plot = route[i + 1] if i + 1 < len(route) else None
                if next_plot is not None and 0 <= next_plot < len(self.plot_centers):
                    next_goal_pose = self.plot_centers[next_plot][0]
                    best_exit_idx = None
                    shortest_len = float('inf')
                    for j in [0, 1]:
                        exit_pose = self.plot_centers[5][j]
                        exit_goal = self.build_goal(*exit_pose)
                        req = GetPlanRequest()
                        req.start.header.frame_id = "map"
                        req.start.header.stamp = rospy.Time.now()
                        req.start.pose = exit_goal.target_pose.pose
                        req.goal = self.build_goal(*next_goal_pose).target_pose
                        req.tolerance = 0.5
                        try:
                            resp = self.make_plan(req)
                            length = sum(math.hypot(
                                resp.plan.poses[k+1].pose.position.x - resp.plan.poses[k].pose.position.x,
                                resp.plan.poses[k+1].pose.position.y - resp.plan.poses[k].pose.position.y
                            ) for k in range(len(resp.plan.poses)-1))
                            rospy.loginfo("Exit %d to next plot: %.2f", j, length)
                            if length < shortest_len:
                                shortest_len = length
                                best_exit_idx = j
                        except Exception as e:
                            rospy.logwarn("Exit %d plan failed: %s", j, str(e))

                    if best_exit_idx is not None:
                        exit_pose = self.plot_centers[5][best_exit_idx]
                        exit_goal = self.build_goal(*exit_pose)
                        if not self.navigate_to_goal("5 exit %d" % best_exit_idx, exit_goal):
                            return
                        rospy.sleep(1.0)
            else:
                if 0 <= plot < len(self.plot_centers):
                    for j, (x, y, theta) in enumerate(self.plot_centers[plot]):
                        goal = self.build_goal(x, y, theta)
                        label = "%d.%d" % (plot, j) if len(self.plot_centers[plot]) > 1 else str(plot)
                        if not self.navigate_to_goal(label, goal):
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

