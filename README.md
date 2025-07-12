# ros2_robots

ROS2 launch file for starting my self designed robots in a variety of configurations. Therefore this is unlikely to be of much general interest (except as an example), unless you have been following my other repos and have chosen to build one or more of the same robots.

## Install

From root ROS2 workspace:
```
git -C src clone https://github.com/jfrancis71/ros2_robots.git
colcon build --symlink-install --packages-select=robots
source ./install/setup.bash
```

## Example Use

```ros2 launch robots robot_launch.py robot:=thomas camera:=true lidar:=true```


## Lidar Notes:

Lidar is model: ldrobot STL-19P

Lidar repo comes from: https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2

For compiling on Ubuntu see following PR:
https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2/pull/24
