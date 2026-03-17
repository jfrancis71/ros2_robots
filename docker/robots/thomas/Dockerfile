FROM ros2_brickpi3

RUN git -C src clone https://github.com/jfrancis71/ros2_mobile_lego
RUN colcon build --symlink-install --packages-select=thomas
RUN git -C src clone https://github.com/jfrancis71/ros2_robots
RUN colcon build --symlink-install --packages-select=robots
RUN git -C src clone https://github.com/jfrancis71/ldlidar_stl_ros2.git
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-select=ldlidar_stl_ros2"
CMD /bin/bash -c "source ./install/setup.bash; ros2 launch robots robot_launch.py robot:=thomas camera:=true lidar:=true"
