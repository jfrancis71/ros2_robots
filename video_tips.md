# Some tips for adding video, eg for a recorded demonstration

To compress an image on the server (change topic according to need):

```
ros2 run image_transport republish --ros-args -p in_transport:=raw -p out_transport:=compressed -r in/robot/image:=/robot/compressed -r out:=/robot/compressed
```


To uncompress an image on the server (change topic according to need):

```
ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw -r in/compressed:=/robot/compressed -r out:=/server/image
```

To publish from an http webcam server (change ip/remap topic according to need):
```
ros2 run image_publisher image_publisher_node --ros-args -p filename:="http://192.168.1.203"
```
