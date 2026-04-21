FROM ros:jazzy-ros-base
RUN apt update
WORKDIR /root/ros2_ws
RUN mkdir src
RUN apt install -y ros-jazzy-cv-bridge
RUN git -C src clone https://github.com/jfrancis71/ros2_stereo_demo
RUN git -C src clone https://github.com/furbrain/image_common
RUN colcon build --symlink-install --packages-select=stereo_demo
RUN colcon build --symlink-install --packages-select=camera_info_manager_py
CMD /bin/bash -c "source ./install/setup.bash; ros2 run stereo_demo stereo_split_node --ros-args \
    --remap /image:=/server/image \
    -p left_camera_info_url:=file:///root/ros2_config/calibration/left.yaml \
    -p right_camera_info_url:=file:///root/ros2_config/calibration/right.yaml"
