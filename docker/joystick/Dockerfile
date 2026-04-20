FROM ros:jazzy-ros-base
RUN apt update
RUN apt install -y ros-jazzy-teleop-twist-joy
WORKDIR /root/ros2_ws
RUN mkdir src
RUN git -C src clone https://github.com/jfrancis71/ros2_joystick_config.git
CMD /bin/bash -c "ros2 launch teleop_twist_joy teleop-launch.py config_filepath:=./src/ros2_joystick_config/joystick_config/config/dual_xeox.config.yaml publish_stamped_twist:=true"
