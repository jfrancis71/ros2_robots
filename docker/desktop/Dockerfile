FROM ros2
RUN apt update
RUN apt install -y ros-jazzy-camera-calibration
RUN apt install -y ros-jazzy-stereo-image-proc
RUN apt install -y ros-jazzy-image-view
WORKDIR /root/ros2_ws
RUN mkdir src
RUN git -C src clone https://github.com/jfrancis71/ros2_joystick_config.git
RUN git -C src clone https://github.com/jfrancis71/ros2_robots
RUN git -C src clone https://github.com/jfrancis71/ros2_stereo_demo
RUN git -C src clone https://github.com/furbrain/image_common
RUN colcon build --symlink-install --packages-select=camera_info_manager_py
RUN colcon build --symlink-install --packages-select=desktop
RUN colcon build --symlink-install --packages-select=stereo_demo
CMD /bin/bash -c "source ./install/setup.bash; ros2 launch desktop desktop_launch.py"
