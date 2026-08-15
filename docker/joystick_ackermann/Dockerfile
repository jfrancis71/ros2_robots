FROM ros:jazzy-ros-base
RUN apt update
RUN apt install -y ros-jazzy-joy
WORKDIR /root/ros2_ws
RUN mkdir src
RUN git -C src clone https://github.com/jfrancis71/ros2_robots.git
RUN git -C src clone https://github.com/jfrancis71/ros2_lego.git
RUN colcon build --symlink-install --packages-select=teleop_ackermann_twist_joy
CMD /bin/bash -c "source ./install/setup.bash; ros2 launch teleop_ackermann_twist_joy teleop-launch.py config_filepath:=/root/ros2_ws/src/ros2_robots/config/ackermann_dual_xeox.config.yaml"
