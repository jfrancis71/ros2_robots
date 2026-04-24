FROM ros:jazzy-ros-base
RUN apt update
RUN apt install -y ros-jazzy-teleop-twist-joy
WORKDIR /root
RUN git clone https://github.com/jfrancis71/ros2_robots.git
CMD /bin/bash -c "ros2 launch teleop_twist_joy teleop-launch.py config_filepath:=/root/ros2_robots/config/dual_xeox.config.yaml publish_stamped_twist:=true"
