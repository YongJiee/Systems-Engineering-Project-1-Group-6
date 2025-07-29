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

class RecoveryNavigator:
    def __init__(self):
        rospy.init_node('recovery_navigator')
        
        # Action client with better error handling
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # Publishers and subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Services with timeout handling
        self.make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
        # State management
        self.current_pose = None
        self.scan_data = None
        self.state = RecoveryState.NAVIGATING
        self.pose_lock = threading.Lock()
        self.scan_lock = threading.Lock()
        
        # Configuration parameters (made configurable)
        self.max_recovery_attempts = rospy.get_param('~max_recovery_attempts', 5)
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.1)
        self.orientation_tolerance = rospy.get_param('~orientation_tolerance', 0.2)
        self.goal_timeout = rospy.get_param('~goal_timeout', 60.0)
        self.min_obstacle_distance = rospy.get_param('~min_obstacle_distance', 0.3)
        self.rotation_speed = rospy.get_param('~rotation_speed', 0.3)
        self.backup_distance = rospy.get_param('~backup_distance', 0.2)
        
        # Recovery behavior configuration - LIMO-optimized order
        self.recovery_behaviors = [
            self.simple_reverse_recovery,      # Most effective for LIMO - just back up
            self.rotate_recovery,              # Rotate to find new path  
            self.clear_costmaps_recovery,      # Clear stale costmap data
            self.wiggle_recovery,              # Small movements to escape
            self.aggressive_clear_recovery     # Last resort
        ]
        
        self.plot_centers = [
            [(0.00, -0.07, 0.0)],               # Home
            [(-1.30, -1.40, 0.0)],              # Leon
            [(-1.43, -0.07, 0.0)],              # Wei Qing
            [(-1.43, 1.24, 0.0)],               # Yong Zun
            [(-0.08, 1.28, math.pi/2)],         # Jia Cheng
            [(1.67, 1.68, -math.pi/2)],         # YT
            [(1.46, -0.07, math.pi / 2)],       # Yong Jie
            [(1.46, -1.20, -math.pi / 2)],      # Gomez
            [(0.00, -1.24, -math.pi / 2)]       # Kieren
        ]

        self.initialize_services()

    def initialize_services(self):
        """Initialize all services with proper error handling and timeouts"""
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

    def stop_robot(self):
        """Safely stop the robot"""
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.1)

    def build_goal(self, x, y, theta):
        """Build a MoveBaseGoal with proper timestamps"""
        q = quaternion_from_euler(0, 0, theta)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = Quaternion(*q)
        return goal

    def check_goal_reached(self, goal):
        """More robust goal checking with orientation"""
        current_pose = self.get_current_pose()
        if not current_pose:
            return False
            
        # Position check
        goal_x = goal.target_pose.pose.position.x
        goal_y = goal.target_pose.pose.position.y
        robot_x = current_pose.position.x
        robot_y = current_pose.position.y
        position_distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        
        # Orientation check
        goal_q = goal.target_pose.pose.orientation
        robot_q = current_pose.orientation
        
        _, _, goal_yaw = euler_from_quaternion([goal_q.x, goal_q.y, goal_q.z, goal_q.w])
        _, _, robot_yaw = euler_from_quaternion([robot_q.x, robot_q.y, robot_q.z, robot_q.w])
        
        yaw_diff = abs(goal_yaw - robot_yaw)
        yaw_diff = min(yaw_diff, 2*math.pi - yaw_diff)  # Handle wraparound
        
        position_ok = position_distance < self.position_tolerance
        orientation_ok = yaw_diff < self.orientation_tolerance
        
        rospy.logdebug("Goal check - Pos: %.3f/%.3f, Ori: %.3f/%.3f", 
                      position_distance, self.position_tolerance,
                      yaw_diff, self.orientation_tolerance)
        
        return position_ok and orientation_ok

    def can_make_plan(self, goal, start_pose=None):
        """Robust path planning check with error handling"""
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
            has_plan = len(resp.plan.poses) > 1  # Need at least start and goal
            rospy.logdebug("Plan check: %d poses", len(resp.plan.poses))
            return has_plan
        except rospy.ServiceException as e:
            rospy.logwarn("Make plan service failed: %s", str(e))
            return False

    def reset_move_base_client(self):
        """Reset the move_base action client to clear any inconsistent states"""
        try:
            # Cancel any existing goals
            current_state = self.client.get_state()
            if current_state in [GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING]:
                rospy.loginfo("Canceling existing goal (state: %d)", current_state)
                self.client.cancel_all_goals()
                
                # Wait for cancellation to complete
                timeout = rospy.Time.now() + rospy.Duration(3.0)
                while rospy.Time.now() < timeout and not rospy.is_shutdown():
                    state = self.client.get_state()
                    if state in [GoalStatus.RECALLED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, 
                                GoalStatus.ABORTED, GoalStatus.SUCCEEDED, GoalStatus.LOST]:
                        break
                    rospy.sleep(0.1)
                
                rospy.loginfo("Goal cancellation completed (final state: %d)", self.client.get_state())
            
        except Exception as e:
            rospy.logwarn("Failed to reset move_base client: %s", str(e))

    def send_goal(self, goal):
        """Send goal with better state monitoring and proper cleanup"""
        # Ensure clean state before sending new goal
        self.reset_move_base_client()
        
        rospy.loginfo("Sending goal to (%.2f, %.2f)", 
                     goal.target_pose.pose.position.x,
                     goal.target_pose.pose.position.y)
        
        self.client.send_goal(goal)
        
        # Monitor goal with periodic checking
        timeout = rospy.Duration(self.goal_timeout)
        check_rate = rospy.Rate(2)  # Check twice per second
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if rospy.Time.now() - start_time > timeout:
                rospy.logwarn("Goal timeout after %.1f seconds", self.goal_timeout)
                self.client.cancel_goal()
                rospy.sleep(0.5)  # Wait for cancellation
                return False
                
            state = self.client.get_state()
            
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached successfully!")
                return True
            elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                rospy.logwarn("Goal failed with state: %s", self.client.get_goal_status_text())
                return False
            elif state == GoalStatus.PREEMPTED:
                rospy.logwarn("Goal was preempted")
                return False
                
            # Check if we're actually close enough despite move_base state
            if self.check_goal_reached(goal):
                rospy.loginfo("Goal reached based on position check!")
                self.client.cancel_goal()
                rospy.sleep(0.3)  # Brief wait for clean cancellation
                return True
                
            check_rate.sleep()
            
        return False

    def analyze_surroundings(self):
        """Analyze laser scan for better recovery decisions"""
        scan_data = self.get_scan_data()
        if not scan_data or not scan_data.ranges:
            return {'front_clear': False, 'sides_clear': False, 'can_rotate': False}
        
        ranges = scan_data.ranges
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment
        
        # Analyze different sectors
        front_sector = []  # ±30 degrees
        left_sector = []   # 30-150 degrees  
        right_sector = []  # -150 to -30 degrees
        
        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r) or r <= 0:
                continue
                
            angle = angle_min + i * angle_increment
            
            if -math.pi/6 <= angle <= math.pi/6:  # Front ±30°
                front_sector.append(r)
            elif math.pi/6 < angle <= 5*math.pi/6:  # Left side
                left_sector.append(r)
            elif -5*math.pi/6 <= angle < -math.pi/6:  # Right side
                right_sector.append(r)
        
        def sector_clear(sector, min_dist):
            if not sector:
                return False
            return min(sector) > min_dist and len([r for r in sector if r > min_dist]) > len(sector) * 0.7
        
        return {
            'front_clear': sector_clear(front_sector, self.min_obstacle_distance),
            'left_clear': sector_clear(left_sector, self.min_obstacle_distance * 0.8),
            'right_clear': sector_clear(right_sector, self.min_obstacle_distance * 0.8),
            'can_rotate': sector_clear(left_sector + right_sector, self.min_obstacle_distance * 0.6)
        }

    def clear_costmaps_recovery(self, goal):
        """Recovery behavior: Clear costmaps"""
        rospy.loginfo("Recovery: Clearing costmaps...")
        self.state = RecoveryState.CLEARING_COSTMAPS
        
        # Ensure clean action client state first
        self.reset_move_base_client()
        
        try:
            self.clear_costmaps()
            rospy.sleep(1.0)  # Wait for costmap refresh
            rospy.loginfo("Costmaps cleared successfully")
            return True
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to clear costmaps: %s", str(e))
            return False

    def rotate_recovery(self, goal):
        """Recovery behavior: Intelligent rotation"""
        rospy.loginfo("Recovery: Rotating to find clear path...")
        self.state = RecoveryState.ROTATING
        
        # Ensure clean action client state before rotation
        self.reset_move_base_client()
        
        surroundings = self.analyze_surroundings()
        if not surroundings['can_rotate']:
            rospy.logwarn("Not enough space to rotate safely")
            return False
        
        # Try different rotation amounts and directions
        rotation_attempts = [
            (1, math.pi/3),    # 60° left
            (-1, math.pi/3),   # 60° right  
            (1, math.pi/2),    # 90° left
            (-1, math.pi/2),   # 90° right
            (1, math.pi),      # 180° left
        ]
        
        for direction, angle in rotation_attempts:
            if self.rotate_and_check(goal, direction, angle):
                return True
                
        rospy.logwarn("Rotation recovery failed to find valid path")
        return False

    def rotate_and_check(self, goal, direction, target_angle):
        """Rotate by specified angle and check for valid path"""
        current_pose = self.get_current_pose()
        if not current_pose:
            return False
            
        # Get initial orientation
        q = current_pose.orientation
        _, _, initial_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        target_yaw = initial_yaw + direction * target_angle
        
        twist = Twist()
        twist.angular.z = direction * self.rotation_speed
        
        rate = rospy.Rate(10)
        timeout = rospy.Time.now() + rospy.Duration(target_angle / self.rotation_speed + 2.0)
        
        while rospy.Time.now() < timeout and not rospy.is_shutdown():
            current_pose = self.get_current_pose()
            if not current_pose:
                break
                
            q = current_pose.orientation
            _, _, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            yaw_diff = abs(current_yaw - target_yaw)
            yaw_diff = min(yaw_diff, 2*math.pi - yaw_diff)
            
            if yaw_diff < 0.1:  # Close enough to target
                break
                
            self.cmd_pub.publish(twist)
            rate.sleep()
        
        self.stop_robot()
        rospy.sleep(0.5)  # Let systems stabilize
        
        # Check if we can now make a plan
        if self.can_make_plan(goal):
            rospy.loginfo("Found valid path after %.1f° rotation", math.degrees(target_angle))
            return True
            
        return False

    def wait_and_retry_recovery(self, goal):
        """Recovery behavior: Simple wait - sometimes the environment changes"""
        rospy.loginfo("Recovery: Waiting for environment to clear...")
        self.reset_move_base_client()
        
        # Just wait - maybe a person will move, door will open, etc.
        rospy.sleep(3.0)
        
        return self.can_make_plan(goal)

    def simple_reverse_recovery(self, goal):
        """Recovery behavior: Simple reverse for LIMO robot"""
        rospy.loginfo("Recovery: Simple reverse maneuver...")
        self.state = RecoveryState.BACKING_UP
        self.reset_move_base_client()
        
        # LIMO-specific parameters
        reverse_distance = 0.03  # 15cm - small but effective for LIMO
        reverse_speed = 0.06     # Slow and controlled
        
        # Basic safety check - ensure we're not completely surrounded
        scan_data = self.get_scan_data()
        if scan_data and scan_data.ranges:
            # Quick check: is there reasonable space behind us?
            ranges = scan_data.ranges
            rear_sector = ranges[-30:] + ranges[:30]  # Rough rear 60° sector
            valid_ranges = [r for r in rear_sector if not (math.isinf(r) or math.isnan(r)) and r > 0]
            
            if valid_ranges and min(valid_ranges) < 0.25:
                rospy.logwarn("Rear appears blocked (%.2f m), skipping reverse", min(valid_ranges))
                return False
        
        # Simple timed reverse
        twist = Twist()
        twist.linear.x = -reverse_speed
        
        reverse_time = reverse_distance / reverse_speed  # About 2.5 seconds
        
        rospy.loginfo("LIMO reversing %.2f m at %.2f m/s for %.1f seconds", 
                     reverse_distance, reverse_speed, reverse_time)
        
        # Execute reverse
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        while (rospy.Time.now() - start_time).to_sec() < reverse_time and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            rate.sleep()
        
        # Stop and settle
        self.stop_robot()
        rospy.sleep(0.3)
        
        # Check if this opened up a path
        success = self.can_make_plan(goal)
        if success:
            rospy.loginfo("Simple reverse successful - path now clear")
        else:
            rospy.loginfo("Reverse completed but path still blocked")
            
        return success

    def wiggle_recovery(self, goal):
        """Recovery behavior: Small oscillating movements to escape local minimum"""
        rospy.loginfo("Recovery: Wiggling to escape local minimum...")
        self.reset_move_base_client()
        
        surroundings = self.analyze_surroundings()
        if not surroundings['can_rotate']:
            rospy.logwarn("Not enough space to wiggle safely")
            return False
        
        # Alternate small rotations and translations
        movements = [
            {'angular': 0.2, 'duration': 1.0},   # Small left turn
            {'linear': 0.05, 'duration': 0.5},   # Tiny forward
            {'angular': -0.4, 'duration': 1.0},  # Small right turn  
            {'linear': 0.05, 'duration': 0.5},   # Tiny forward
            {'angular': 0.2, 'duration': 1.0},   # Back to roughly original heading
        ]
        
        rate = rospy.Rate(10)
        
        for movement in movements:
            twist = Twist()
            if 'angular' in movement:
                twist.angular.z = movement['angular']
            if 'linear' in movement:
                twist.linear.x = movement['linear']
            
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < movement['duration']:
                self.cmd_pub.publish(twist)
                rate.sleep()
            
            # Check if path is available after each movement
            self.stop_robot()
            rospy.sleep(0.2)
            
            if self.can_make_plan(goal):
                rospy.loginfo("Wiggle recovery found path after movement")
                return True
        
        return self.can_make_plan(goal)

    def aggressive_clear_recovery(self, goal):
        """Recovery behavior: Aggressive costmap clearing with small movement"""
        rospy.loginfo("Recovery: Aggressive clearing with repositioning...")
        
        # Ensure clean action client state
        self.reset_move_base_client()
        
        # Clear costmaps multiple times
        for _ in range(3):
            try:
                self.clear_costmaps()
                rospy.sleep(0.5)
            except:
                pass
        
        # Small random movement to change robot position slightly
        twist = Twist()
        twist.linear.x = 0.05
        twist.angular.z = 0.1
        
        for _ in range(10):
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
        
        self.stop_robot()
        rospy.sleep(1.0)
        
        return self.can_make_plan(goal)

    def navigate_to_goal(self, plot_name, goal):
        """Navigate to goal with systematic recovery"""
        rospy.loginfo("Navigating to plot %s at (%.2f, %.2f)", 
                     plot_name, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
        
        # Try direct navigation first
        if self.can_make_plan(goal):
            if self.send_goal(goal):
                rospy.loginfo("Successfully reached plot %s", plot_name)
                return True
        
        # If direct navigation fails, try recovery behaviors
        for attempt in range(self.max_recovery_attempts):
            rospy.loginfo("Recovery attempt %d/%d for plot %s", 
                         attempt + 1, self.max_recovery_attempts, plot_name)
            
            # Try each recovery behavior
            recovery_success = False
            for i, recovery_behavior in enumerate(self.recovery_behaviors):
                rospy.loginfo("Trying recovery behavior %d", i + 1)
                
                if recovery_behavior(goal):
                    # Recovery behavior succeeded, try navigation again
                    if self.can_make_plan(goal):
                        if self.send_goal(goal):
                            rospy.loginfo("Reached plot %s after recovery", plot_name)
                            return True
                    recovery_success = True
                    break
                
                rospy.sleep(0.5)  # Brief pause between recovery attempts
            
            if not recovery_success:
                rospy.logwarn("All recovery behaviors failed for attempt %d", attempt + 1)
            
            rospy.sleep(1.0)  # Pause between recovery attempts
        
        self.state = RecoveryState.FAILED
        rospy.logerr("Failed to reach plot %s after %d recovery attempts", 
                    plot_name, self.max_recovery_attempts)
        return False

    def run(self):
        """Main execution loop"""
        route = rospy.get_param("~options", [0])  # Default to home
        rospy.loginfo("Executing route: %s", route)
        
        success_count = 0
        total_plots = len(route)
        
        for plot_idx in route:
            if not (0 <= plot_idx < len(self.plot_centers)):
                rospy.logwarn("Invalid plot index: %d, skipping...", plot_idx)
                continue
            
            goal_pose = self.plot_centers[plot_idx][0]
            goal = self.build_goal(*goal_pose)
            
            if self.navigate_to_goal(str(plot_idx), goal):
                success_count += 1
                rospy.sleep(2.0)  # Pause at each successful plot
            else:
                # Ask user if they want to continue or abort
                rospy.logwarn("Failed to reach plot %d. Continuing to next plot...", plot_idx)
                continue
        
        rospy.loginfo("Navigation completed: %d/%d plots reached successfully", 
                     success_count, total_plots)
        
        if success_count == total_plots:
            rospy.loginfo("All plots reached successfully!")
        else:
            rospy.logwarn("Some plots were not reached. Check logs for details.")

if __name__ == '__main__':
    try:
        navigator = RecoveryNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted by user")
    except Exception as e:
        rospy.logerr("Navigation failed with error: %s", str(e))
        raise
