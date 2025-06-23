<<<<<<< HEAD
<<<<<<< HEAD
# Systems-Engineering-Project-1-Group-6-
This repository documents my Systems Engineering project at university, using the LIMO mobile robot with ROS1 Melodic to design, build, and test an autonomous navigation system in real-world conditions.
=======
## start the motor driver
=======
# limo_ros
This repository contains ROS packages for limo. 
>>>>>>> refactor code

<img src="limo_description/img/limo.jpg" width="640" height="208" /> 

## Packages
 
 
* limo_base: ROS wrapper for limo
* limo_bringup: launch and configuration files to start ROS nodes


## Build from source code
Clone the repository and catkin_make:
```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/agilexrobotics/limo_ros.git
    $ cd ..
    $ catkin_make
```


## Usage

* Start the base node for limo

    ```
    $ roslaunch limo_bringup limo_start.launch
    ```

<<<<<<< HEAD
```shell
# this will launch the amcl and move_base node
roslaunch limo_bringup limo_navigation.launch
```
<<<<<<< HEAD
>>>>>>> add readme
=======
## build the 2d map
=======
>>>>>>> refactor code

* Start the keyboard tele-op node

<<<<<<< HEAD
And then use map_server to save the map, see [map_server](http://wiki.ros.org/map_server)
>>>>>>> change readme
=======
    ```
    $ roslaunch limo_bringup limo_teleop_keyboard.launch
    ```
>>>>>>> refactor code
