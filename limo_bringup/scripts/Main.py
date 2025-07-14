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
        self.yaw_tolerance = 0.2  # radians

        self.plot_centers = [
            (0.0, 0.0, 0.0),                # 0 - Home
            (0.0, 0.0, 0.0),                  # 1
            (2.5, 0.2, math.pi / 2),          # 2
            (1.7, 0.5, 3.1),                  # 3 (Yong Zun)
            (0.0, 1.35, math.pi / 2),         # 4 (Jia Cheng)
            (3.0, 2.0, 0.0),                  # 5
            (1.55,0.0, math.pi / 2),         # 6 (YJ)
            (0.0, 0.0, -math.pi / 4),         # 7
            (0.0, 0.0, -math.pi/2)               # 8
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

    def get_goals_from_params(self):
        indices = rospy.get_param("~options", [1, 2, 3])
        self.goal_list = []

        for i in indices:
            if 0 <= i < len(self.plot_centers):
                x, y, theta = self.plot_centers[i]
                q = quaternion_from_euler(0, 0, theta)
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = x
                goal.target_pose.pose.position.y = y
                goal.target_pose.pose.orientation = Quaternion(*q)
                self.goal_list.append((i, goal))
                rospy.loginfo("Goal #%d added: (%.2f, %.2f, %.2f rad)", i, x, y, theta)
            else:
                rospy.logwarn("Invalid goal index: %d", i)

    def send_goal(self, goal):
        self.client.send_goal(goal)
        timeout = rospy.Duration(45)
        finished = self.client.wait_for_result(timeout)
        if finished:
            state = self.client.get_state()
            if state == 3:
                rospy.loginfo("Goal reached successfully!")
                return True
            else:
                rospy.logwarn("Goal failed with state: %d", state)
        else:
            rospy.logwarn("Goal timed out")
            self.client.cancel_goal()
        return self.check_goal_reached(goal)

    def check_goal_reached(self, goal):
        if not self.current_pose:
            return False
        goal_x = goal.target_pose.pose.position.x
        goal_y = goal.target_pose.pose.position.y
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        goal_yaw = self.yaw_from_quaternion(goal.target_pose.pose.orientation)
        robot_yaw = self.yaw_from_quaternion(self.current_pose.orientation)
        yaw_error = abs((goal_yaw - robot_yaw + math.pi) % (2 * math.pi) - math.pi)
        rospy.loginfo("Distance to goal: %.2f m | Yaw error: %.2f rad", distance, yaw_error)
        if distance < self.position_tolerance and yaw_error < self.yaw_tolerance:
            rospy.loginfo("Goal reached within tolerance!")
            return True
        elif distance < self.position_tolerance:
            rospy.loginfo("Position reached, but orientation off by %.2f rad", yaw_error)
            return True
        return False

    def yaw_from_quaternion(self, q):
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def can_make_plan(self, goal):
        if not self.current_pose:
            rospy.logwarn("No current pose available for planning.")
            return False
        req = GetPlanRequest()
        req.start.header.frame_id = "map"
        req.start.header.stamp = rospy.Time.now()
        req.start.pose.position = self.current_pose.position
        req.start.pose.orientation = self.current_pose.orientation
        req.goal = goal.target_pose
        req.tolerance = 0.5
        try:
            resp = self.make_plan(req)
            has_plan = len(resp.plan.poses) > 0
            rospy.loginfo("Plan check: %s (%d poses)", "Valid" if has_plan else "Invalid", len(resp.plan.poses))
            return has_plan
        except Exception as e:
            rospy.logwarn("make_plan service failed: %s", str(e))
            return False

    def has_room_to_rotate(self, min_clearance=0.25):
        if not self.scan_data or not self.scan_data.ranges:
            rospy.logwarn("No scan data available for rotation check.")
            return False
        ranges = self.scan_data.ranges
        valid_ranges = [r for r in ranges if not math.isinf(r) and not math.isnan(r) and r > 0]
        if not valid_ranges:
            rospy.logwarn("All scan readings are invalid.")
            return False
        min_distance = min(valid_ranges)
        can_rotate = min_distance > min_clearance
        rospy.loginfo("Min clearance: %.2f m, Required: %.2f m, Can rotate: %s", 
                     min_distance, min_clearance, can_rotate)
        return can_rotate

    def clear_costmap(self):
        try:
            self.clear_costmaps()
            rospy.loginfo("Costmaps cleared successfully.")
            return True
        except Exception as e:
            rospy.logwarn("Failed to clear costmaps: %s", str(e))
            return False

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
        steps = max(3, 8 - self.recovery_attempts)
        for _ in range(steps):
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.5)

    def recovery_behavior(self, goal):
        rospy.loginfo("Starting recovery behavior (attempt %d/%d)...", 
                     self.recovery_attempts + 1, self.max_recovery_attempts)

        self.clear_costmap()

        if self.has_room_to_rotate():
            self.rotate_until_path_found(goal)
        else:
            self.reverse_slightly()

        if self.can_make_plan(goal):
            rospy.loginfo("Path found after recovery action. Waiting 1 second before retrying move...")
            rospy.sleep(1.0)

        self.recovery_attempts += 1
        rospy.loginfo("Recovery behavior completed, will retry sending goal...")

    def navigate_to_goal(self, idx, goal):
        rospy.loginfo("Navigating to plot %d", idx)
        self.recovery_attempts = 0
        while not rospy.is_shutdown() and self.recovery_attempts < self.max_recovery_attempts:
            if self.can_make_plan(goal):
                success = self.send_goal(goal)
                if success:
                    rospy.loginfo("Successfully reached plot %d!", idx)
                    return True
                else:
                    rospy.logwarn("Failed to reach plot %d, attempting recovery...", idx)
                    self.recovery_behavior(goal)
            else:
                rospy.logwarn("No valid plan to plot %d, attempting recovery...", idx)
                self.recovery_behavior(goal)
        rospy.logerr("Failed to reach plot %d after %d recovery attempts", idx, self.max_recovery_attempts)
        return False

    def run(self):
        self.get_goals_from_params()
        if not self.goal_list:
            rospy.logerr("No valid goals found!")
            return
        rospy.loginfo("Starting navigation sequence with %d goals", len(self.goal_list))
        for idx, goal in self.goal_list:
            if not self.navigate_to_goal(idx, goal):
                rospy.logerr("Mission failed at plot %d", idx)
                break
            rospy.sleep(1.0)
        rospy.loginfo("Navigation sequence completed!")

if __name__ == '__main__':
    try:
        navigator = RecoveryNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted by user")
    except Exception as e:
        rospy.logerr("Navigation failed with error: %s", str(e))

