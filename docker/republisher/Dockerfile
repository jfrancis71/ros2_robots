FROM ros:jazzy-ros-base
RUN apt update
RUN apt install -y ros-jazzy-image-publisher
RUN apt install -y ros-jazzy-compressed-image-transport
WORKDIR /root/ros2_ws
CMD /bin/bash -c "ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw -r in/compressed:=/robot/compressed -r out:=/server/image"
