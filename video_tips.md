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
ros2 run image_publisher image_publisher_node --ros-args -p filename:="http://192.168.1.246/video.mjpg"
```

Note, you may receive error message:
```
[ WARN:0@0.067] global ./modules/imgcodecs/src/loadsave.cpp (239) findDecoder imread_('http://192.168.1.246/video.mjpg'): can't open/read file: check file path/integrity
```
which is misleading and can be ignored. You may wish to set ipad/iphone auto lock to never (temporarily while using video)

Tested on iPad, SKJM LLC: ipCam 1.2.2
