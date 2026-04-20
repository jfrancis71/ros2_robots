FROM ros:jazzy-ros-base
RUN apt update
RUN apt install -y ros-jazzy-nav2-bringup
WORKDIR /root/ros2_ws
RUN mkdir src
RUN git -C src clone https://github.com/jfrancis71/ros2_nav2_demo
CMD /bin/bash -c "ros2 launch nav2_bringup bringup_launch.py map:=/root/ros2_config/my_house.yaml params_file:=./src/ros2_nav2_demo/nav2_params.yaml"
