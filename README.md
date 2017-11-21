# TurtleBot Simple Walker

### Overview

- A ROS package with a simple walker algorithm for a TurtleBot to move like a Roomba vacuum, simulated in Gazebo. The robot will move forward until detecting an obstacle, turn in place, and continue moving forward.

- The included node publishes to the */cmd_vel_mux/input/teleop* to drive the robot (publishing to the */mobile_base/commands/velocity* topic would work as well). It also subscribes to the */scan* topic to get sensor information.


### Dependencies
- ROS Kinetic
- Gazebo
- catkin
- turtlebot_gazebo
- roscpp package
- geometry_msgs package



### How to Build
- Create the catkin workspace and build with:
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone --recursive https://github.com/zzjkf2009/TurtleBot-Roomba
cd ..
catkin_make
```

### Running 
- To run the simulation, use the *Botwalker.launch* launch file.

```
source devel/setup.bash
roslaunch turtlebotwalker Botwalker.launch
```

This launches gazebo with the TurtleBot in the *corridor.world* world. Also, a new terminal will open with current information about the TurtleBot. The distance to an object in front of the robot is printed, along with the current action, *Forward* or *Turn*.

![gazebo](https://github.com/zzjkf2009/TurtleBot-Roomba/blob/master/result/world.png)

The robot will move forward until detecting an obstacle close enough (threshold is .7) and then turn left in place until it is safe to move forward again. 




### Recording Bag Files
- By default, the launch file will not record a bag file. Recording can be enabled by adding a *startRecord:=true* argument to the launch command.
```
roslaunch turtlebot_roomba turtleRoomba.launch startRecord:=true
```

This will be the run the same as described previously, but a bag file *.bag* will be recorded. Ctrl+C in the terminal where the roslaunch command was done will stop the simulation and recording. The bag file will be saved originally in the *.ros* directory, not the current directory. By adding *$(find turtlebotwalker)*, it will then be saved in turtlebotwalker package folder.



#### Bag File Info
- Information can be seen about the recording using *rosboag info*. All topics will be recorded except /camera/* topics.

To inspect the bag file,
```
rosbag info BagResult.bag
```

![rosbag-info](https://github.com/zzjkf2009/TurtleBot-Roomba/blob/master/result/BagResult_info.png)


#### Bag File Playback
- The bag file can be played back using *rosbag play*. Gazebo should not be running for the playback. To playback, open a terminal and run roscore, then open another terminal and navigate to the *.ros* directory. The sensor data will be published again like when the simulation was running.
```
rosbag play BagResult.bag
```
