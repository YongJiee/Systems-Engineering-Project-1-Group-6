#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
import threading
from enum import Enum
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from actionlib_msgs.msg import GoalStatus

class RecoveryState(Enum):
    NAVIGATING = 1
    CLEARING_COSTMAPS = 2
    ROTATING = 3
    BACKING_UP = 4
    FAILED = 5
    RETURNING_HOME = 6

class RecoveryNavigator:
    def __init__(self):
        rospy.init_node('recovery_navigator')

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        self.current_pose = None
        self.scan_data = None
        self.state = RecoveryState.NAVIGATING
        self.pose_lock = threading.Lock()
        self.scan_lock = threading.Lock()

        self.max_recovery_attempts = rospy.get_param('~max_recovery_attempts', 4)
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.2)
        self.orientation_tolerance = rospy.get_param('~orientation_tolerance', 0.2)
        self.goal_timeout = rospy.get_param('~goal_timeout', 60.0)
        self.min_obstacle_distance = rospy.get_param('~min_obstacle_distance', 0.2)
        self.rotation_speed = rospy.get_param('~rotation_speed', 0.3)
        self.backup_distance = rospy.get_param('~backup_distance', 0.2)
        
        # New parameters for retry logic
        self.max_goal_attempts = rospy.get_param('~max_goal_attempts', 2)
        self.max_home_retry_attempts = rospy.get_param('~max_home_retry_attempts', 999)
        self.ignore_orientation = rospy.get_param('~ignore_orientation', True)

        self.recovery_behaviors = [
            self.clear_costmaps_recovery,
            self.rotate_recovery,
            self.backup_recovery,
            self.aggressive_clear_recovery
        ]

        self.plot_centers = [
            [(0.00, -0.07, 0.0)],  # Home - Index 0
            [(-1.21, -1.50, 0.0)], # Leon
            [(-1.32, -0.15, 0.0)], # Wei Qing
            [(-1.48, 1.07, 0.0)],  # Yong Zun
            [(-0.15, 1.21, 0.0)], # Jia Cheng
            [(1.69, 1.62, 0.0)], # YT
            [(1.46, -0.07, 0.0)], # Yong Jie
            [(1.46, -1.20, 0.0)], # Gomez
            [(0.00, -1.18, 0.0)]  # Kieren
        ]

        self.home_index = 0  # Index of home position
        self.initialize_services()

    def odom_callback(self, msg):
        with self.pose_lock:
            self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        with self.scan_lock:
            self.scan_data = msg

    def get_current_pose(self):
        with self.pose_lock:
            return self.current_pose

    def get_scan_data(self):
        with self.scan_lock:
            return self.scan_data

    def initialize_services(self):
        rospy.loginfo("Waiting for move_base and services...")
        try:
            if not self.client.wait_for_server(rospy.Duration(10.0)):
                raise RuntimeError("move_base action server not available")
            self.make_plan.wait_for_service(timeout=10.0)
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=10.0)
            rospy.loginfo("Connected to all services successfully.")
        except Exception as e:
            rospy.logerr("Failed to connect to services: %s", str(e))
            raise

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.1)

    def build_goal(self, x, y, theta=0.0):
        q = quaternion_from_euler(0, 0, theta)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = Quaternion(*q)
        return goal

    def check_goal_reached(self, goal):
        current_pose = self.get_current_pose()
        if not current_pose:
            return False

        goal_x = goal.target_pose.pose.position.x
        goal_y = goal.target_pose.pose.position.y
        robot_x = current_pose.position.x
        robot_y = current_pose.position.y
        position_distance = math.hypot(goal_x - robot_x, goal_y - robot_y)

        # Check if we should ignore orientation
        if self.ignore_orientation:
            return position_distance < self.position_tolerance
        else:
            # Original orientation check
            goal_q = goal.target_pose.pose.orientation
            robot_q = current_pose.orientation

            _, _, goal_yaw = euler_from_quaternion([goal_q.x, goal_q.y, goal_q.z, goal_q.w])
            _, _, robot_yaw = euler_from_quaternion([robot_q.x, robot_q.y, robot_q.z, robot_q.w])
            yaw_diff = abs(goal_yaw - robot_yaw)
            yaw_diff = min(yaw_diff, 2*math.pi - yaw_diff)

            return position_distance < self.position_tolerance and yaw_diff < self.orientation_tolerance

    def can_make_plan(self, goal, start_pose=None):
        current_pose = start_pose or self.get_current_pose()
        if not current_pose:
            return False

        req = GetPlanRequest()
        req.start.header.frame_id = "map"
        req.start.header.stamp = rospy.Time.now()
        req.start.pose = current_pose
        req.goal = goal.target_pose
        req.tolerance = 0.5

        try:
            resp = self.make_plan(req)
            return len(resp.plan.poses) > 1
        except rospy.ServiceException:
            return False

    def send_goal(self, goal):
        self.client.send_goal(goal)
        timeout = rospy.Duration(self.goal_timeout)
        check_rate = rospy.Rate(2)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if rospy.Time.now() - start_time > timeout:
                self.client.cancel_goal()
                return False

            state = self.client.get_state()

            if state == GoalStatus.SUCCEEDED:
                return True
            elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED]:
                return False

            if self.check_goal_reached(goal):
                self.client.cancel_goal()
                return True

            check_rate.sleep()

        return False

    def analyze_surroundings(self):
        scan_data = self.get_scan_data()
        if not scan_data or not scan_data.ranges:
            return {'front_clear': False, 'sides_clear': False, 'can_rotate': False}

        ranges = scan_data.ranges
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment

        front = []
        left = []
        right = []

        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r) or r <= 0:
                continue

            angle = angle_min + i * angle_increment
            if -math.pi/6 <= angle <= math.pi/6:
                front.append(r)
            elif math.pi/6 < angle <= 5*math.pi/6:
                left.append(r)
            elif -5*math.pi/6 <= angle < -math.pi/6:
                right.append(r)

        def sector_clear(sector, min_dist):
            return sector and min(sector) > min_dist and sum(r > min_dist for r in sector) > 0.7 * len(sector)

        return {
            'front_clear': sector_clear(front, self.min_obstacle_distance),
            'left_clear': sector_clear(left, self.min_obstacle_distance * 0.8),
            'right_clear': sector_clear(right, self.min_obstacle_distance * 0.8),
            'can_rotate': sector_clear(left + right, self.min_obstacle_distance * 0.6)
        }

    def clear_costmaps_recovery(self, goal):
        rospy.loginfo("Recovery: Backing up before clearing costmaps...")
        self.state = RecoveryState.BACKING_UP

        surroundings = self.analyze_surroundings()
        if not surroundings['front_clear'] and not surroundings['left_clear'] and not surroundings['right_clear']:
            rospy.logwarn("Completely surrounded, skipping backup before costmap clear")
        else:
            twist = Twist()
            twist.linear.x = -0.01
            backup_duration = self.backup_distance / 0.1
            rate = rospy.Rate(10)
            for _ in range(int(backup_duration * 10)):
                self.cmd_pub.publish(twist)
                rate.sleep()
            self.stop_robot()
            rospy.sleep(0.5)

        rospy.loginfo("Recovery: Clearing costmaps...")
        self.state = RecoveryState.CLEARING_COSTMAPS
        try:
            self.clear_costmaps()
            rospy.sleep(1.0)
            return True
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to clear costmaps: %s", str(e))
            return False

    def rotate_recovery(self, goal):
        rospy.loginfo("Recovery: Rotating to find clear path...")
        self.state = RecoveryState.ROTATING
        surroundings = self.analyze_surroundings()
        if not surroundings['can_rotate']:
            return False

        for direction, angle in [(1, math.pi/2), (-1, math.pi/2), (1, math.pi), (-1, math.pi)]:
            if self.rotate_and_check(goal, direction, angle):
                return True

        return False

    def rotate_and_check(self, goal, direction, target_angle):
        current_pose = self.get_current_pose()
        if not current_pose:
            return False

        q = current_pose.orientation
        _, _, initial_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        target_yaw = initial_yaw + direction * target_angle

        twist = Twist()
        twist.angular.z = direction * self.rotation_speed
        rate = rospy.Rate(10)
        timeout = rospy.Time.now() + rospy.Duration(target_angle / self.rotation_speed + 2.0)

        while rospy.Time.now() < timeout and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            rate.sleep()

        self.stop_robot()
        rospy.sleep(0.5)

        return self.can_make_plan(goal)

    def backup_recovery(self, goal):
        rospy.loginfo("Recovery: Backing up...")
        self.state = RecoveryState.BACKING_UP
        surroundings = self.analyze_surroundings()
        if not surroundings['front_clear'] and not surroundings['left_clear'] and not surroundings['right_clear']:
            return False

        twist = Twist()
        twist.linear.x = -0.1
        duration = self.backup_distance / 0.1
        rate = rospy.Rate(10)
        for _ in range(int(duration * 10)):
            self.cmd_pub.publish(twist)
            rate.sleep()

        self.stop_robot()
        rospy.sleep(0.5)
        return self.can_make_plan(goal)

    def aggressive_clear_recovery(self, goal):
        for _ in range(3):
            try:
                self.clear_costmaps()
                rospy.sleep(0.5)
            except:
                pass

        twist = Twist()
        twist.linear.x = 0.05
        twist.angular.z = 0.1
        for _ in range(10):
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)

        self.stop_robot()
        rospy.sleep(1.0)
        return self.can_make_plan(goal)

    def return_to_home(self):
        """Return to home position with recovery attempts"""
        rospy.loginfo("Returning to home position...")
        self.state = RecoveryState.RETURNING_HOME
        
        home_pose = self.plot_centers[self.home_index][0]
        home_goal = self.build_goal(*home_pose)
        
        # Try to return home with recovery behaviors
        if self.can_make_plan(home_goal):
            if self.send_goal(home_goal):
                rospy.loginfo("Successfully returned to home")
                return True

        # If direct navigation fails, try recovery behaviors
        for attempt in range(self.max_recovery_attempts):
            rospy.loginfo("Home return recovery attempt %d/%d", attempt + 1, self.max_recovery_attempts)
            for recovery_behavior in self.recovery_behaviors:
                if recovery_behavior(home_goal):
                    if self.can_make_plan(home_goal):
                        if self.send_goal(home_goal):
                            rospy.loginfo("Successfully returned to home after recovery")
                            return True
                    break
            rospy.sleep(1.0)

        rospy.logwarn("Failed to return to home after all recovery attempts")
        return False

    def navigate_to_goal_with_retry(self, plot_name, goal):
        """Navigate to goal with retry logic including home return"""
        rospy.loginfo("Navigating to plot %s with retry logic", plot_name)
        
        # First attempt: try to reach goal directly (up to max_goal_attempts times)
        for attempt in range(self.max_goal_attempts):
            rospy.loginfo("Direct navigation attempt %d/%d to plot %s", 
                         attempt + 1, self.max_goal_attempts, plot_name)
            
            if self.navigate_to_goal(plot_name, goal):
                return True
            
            rospy.logwarn("Failed direct navigation attempt %d/%d to plot %s", 
                         attempt + 1, self.max_goal_attempts, plot_name)
            rospy.sleep(1.0)
        
        rospy.logwarn("Failed to reach plot %s after %d direct attempts. Returning to home...", 
                     plot_name, self.max_goal_attempts)
        
        # Return to home
        if not self.return_to_home():
            rospy.logerr("Failed to return to home. Skipping plot %s", plot_name)
            return False
        
        rospy.sleep(2.0)  # Wait a bit at home
        
        # Second attempt: try from home (up to max_home_retry_attempts times)
        for attempt in range(self.max_home_retry_attempts):
            rospy.loginfo("Home-based navigation attempt %d/%d to plot %s", 
                         attempt + 1, self.max_home_retry_attempts, plot_name)
            
            if self.navigate_to_goal(plot_name, goal):
                return True
            
            rospy.logwarn("Failed home-based navigation attempt %d/%d to plot %s", 
                         attempt + 1, self.max_home_retry_attempts, plot_name)
            
            # Return to home between attempts (except for the last attempt)
            if attempt < self.max_home_retry_attempts - 1:
                rospy.loginfo("Returning to home before next attempt...")
                if not self.return_to_home():
                    rospy.logerr("Failed to return to home during retry sequence")
                    break
                rospy.sleep(1.0)
        
        rospy.logerr("Failed to reach plot %s after all retry attempts. Moving to next plot.", plot_name)
        return False

    def navigate_to_goal(self, plot_name, goal):
        """Original navigation method with recovery behaviors"""
        rospy.loginfo("Navigating to plot %s", plot_name)
        self.state = RecoveryState.NAVIGATING
        
        if self.can_make_plan(goal):
            if self.send_goal(goal):
                return True

        for attempt in range(self.max_recovery_attempts):
            rospy.loginfo("Recovery attempt %d/%d for plot %s", 
                         attempt + 1, self.max_recovery_attempts, plot_name)
            for recovery_behavior in self.recovery_behaviors:
                if recovery_behavior(goal):
                    if self.can_make_plan(goal):
                        if self.send_goal(goal):
                            return True
                    break
            rospy.sleep(1.0)

        self.state = RecoveryState.FAILED
        return False

    def run(self):
        route = rospy.get_param("~options", [0])
        rospy.loginfo("Executing route: %s", route)
        rospy.loginfo("Max goal attempts: %d, Max home retry attempts: %d", 
                     self.max_goal_attempts, self.max_home_retry_attempts)
        rospy.loginfo("Ignore orientation: %s", self.ignore_orientation)

        success = 0
        for plot_idx in route:
            if not (0 <= plot_idx < len(self.plot_centers)):
                rospy.logwarn("Invalid plot index: %d. Skipping.", plot_idx)
                continue
                
            goal_pose = self.plot_centers[plot_idx][0]
            goal = self.build_goal(*goal_pose)
            
            if self.navigate_to_goal_with_retry(str(plot_idx), goal):
                success += 1
                rospy.loginfo("Successfully reached plot %d", plot_idx)
                rospy.sleep(2.0)  # Wait at the plot
            else:
                rospy.logwarn("Failed to reach plot %d after all attempts", plot_idx)

        rospy.loginfo("Navigation complete: %d/%d plots reached", success, len(route))

if __name__ == '__main__':
    try:
        navigator = RecoveryNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Navigation failed with error: %s", str(e))
        raise
