FROM ros:jazzy-ros-base
RUN apt update
RUN apt install -y ros-jazzy-stereo-image-proc
CMD /bin/bash -c "ros2 launch stereo_image_proc stereo_image_proc.launch.py sgbm_mode:=2"
