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

```docker -H ssh://julian@brickpi3 run -it --rm --privileged --network=host --ipc=host robot_thomas```

Brings up joystick and stero pipeline:

```docker run -it --rm --privileged --network=host --ipc=host -v ros2_ws:/root/ros2_ws --volume=ros2_config:/root/ros2_config ros2_desktop```

Brings up NAV2:

```docker run -it --rm --network=host --ipc=host --volume=ros2_config:/root/ros2_config ros2_nav2```

Bring up general:

```docker run -it --rm --privileged --network=host --ipc=host -v ros2_ws:/root/ros2_ws -v ros2_config:/root/ros2_config     -v $HOME/.gitconfig:/root/.gitconfig -v $HOME/.git-credentials:/root/.git-credentials     -v="$XAUTHORITY:$XAUTHORITY" --env="XAUTHORITY=$XAUTHORITY"  --env="DISPLAY=$DISPLAY" ros2_desktop /bin/bash```

## Lidar Notes:

Lidar is model: ldrobot STL-19P

Lidar repo comes from: https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2

For compiling on Ubuntu see following PR:
https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2/pull/24
