FROM ros:jazzy-ros-base
RUN apt update
WORKDIR /root/ros2_ws
RUN mkdir src
RUN git -C src clone https://github.com/jfrancis71/ldlidar_stl_ros2.git
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-select=ldlidar_stl_ros2"
RUN git -C src clone https://github.com/jfrancis71/ros2_robots
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-select robots"
CMD /bin/bash -c "source ./install/setup.bash; ros2 launch robots lidar_launch.py"
