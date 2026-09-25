FROM ros:jazzy-ros-base
RUN apt update
RUN apt install -y ros-jazzy-usb-cam
#WORKDIR /root/ros2_ws
# Note usb-cam node crashes unless image dimensions are set.
# I suspect it defaults to some convention eg 640x480 and then crashes when the camera
# cannot support those dimensions, especially a stereo camera
CMD /bin/bash -c "ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:=mjpeg2rgb -p image_height:=240 -p image_width:=640 -r /image_raw:=/robot/image -r /image_raw/compressed:=/robot/compressed"
