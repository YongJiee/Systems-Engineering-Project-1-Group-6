# RSE2107A - Systems Engineering Project 1 (AY2024/25 T3)

Welcome to our project portfolio! We are **Group 6** from the **Robotics Systems Engineering** course, and this repository documents the development and implementation of an **autonomous navigation system** using the **LIMO Robot** and **ROS1 Melodic**.

---

## Team Members

- Aloysius Ho Jun Sheng  
- Alson Lim Chin Meng  
- Beckham Benny Ross  
- Jin Ziyu  
- Ng Si En Jennifer  
- **Tan Yong Jie** (Team Leader)

---

##  Project Focus: LIMO Robot Autonomous Navigation

Our goal was to develop a reliable indoor navigation system using the AgileX LIMO robot. The key objectives included:

-  **Autonomous Navigation**: Move to a target goal using ROS `move_base`
-  **Mapping & Localization**: Perform SLAM with `gmapping` or `rtabmap_ros`, and localize with `amcl`
-  **Obstacle Avoidance**: Leverage LIDAR and costmaps for real-time obstacle detection and avoidance
-  **Command Handling**: Support both manual teleoperation and fully autonomous command execution via ROS

---

## Skills & Tools Used

| Category           | Tools & Technologies                              |
|--------------------|---------------------------------------------------|
| **Robot Platform** | AgileX LIMO Robot (4WD)                           |
| **Middleware**     | ROS1 Melodic                                      |
| **Languages**      | Python                                            |
| **Simulation**     | Gazebo, RViz                                      |
| **Mapping**        | `gmapping`, `rtabmap_ros`                         |
| **Localization**   | `amcl`                                            |
| **Navigation**     | `move_base`, `costmap_2d`                         |
| **Control**        | `Teleop`, `cmd_vel`, Autonomous Path Planning     |
| **Other Tools**    | `TF`, `roslaunch`, `rosbag`, `rqt_graph`          |

---

## Launch Instructions
- rosrun tf tf_echo /map /base_link

### RTAB-Map Mapping Mode
- roslaunch limo_bringup limo_start.launch pub_odom_tf:=false
- roslaunch limo_bringup limo_mapping_rtabmap.launch 
- roslaunch limo_bringup rtabmap_rviz.launch

### RTAB-Map Navigation Mode (with Localization)
- roslaunch limo_bringup limo_start.launch pub_odom_tf:=false
- roslaunch limo_bringup limo_navigation_rtabmap.launch
- roslaunch limo_bringup rtabmap_rviz.launch

### Running Autonomous Behavior Script
- rosrun limo_bringup Main.py _options:="[6]"

