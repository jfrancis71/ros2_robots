# ros2_robots

ROS2 launch file for starting my self designed robots in a variety of configurations. Therefore this is unlikely to be of much general interest (except as an example), unless you have been following my other repos and have chosen to build one or more of the same robots.


## Example Use

To launch Thomas the robot (on brickpi3):
```
DOCKER_HOST="ssh://julian@brickpi3" docker compose -f ./docker-compose/thomas/docker-compose.yaml up -d
```

To stop Thomas the robot (on brickpi3):
```
DOCKER_HOST="ssh://julian@brickpi3" docker compose -f ./docker-compose/thomas/docker-compose.yaml down
```

Brings up services:
```
docker compose -f ./docker-compose/server/docker-compose.yaml up -d
```

Shuts down services:
```
docker compose -f ./docker-compose/server/docker-compose.yaml down
```


Bring up rviz2:

```
docker run -it --rm --network=host --ipc=host -v="$XAUTHORITY:$XAUTHORITY" --env="XAUTHORITY=$XAUTHORITY"  --env="DISPLAY=$DISPLAY" rviz2
```

## Lidar Notes:

Lidar is model: ldrobot STL-19P

Lidar repo comes from: https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2

For compiling on Ubuntu see following PR:
https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2/pull/24
