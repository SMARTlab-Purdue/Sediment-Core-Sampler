## Dockerized ROS Services

To build Wabash ROS image in Docker,
```
$ ./build.sh
```

To run ROS core,
```
$ ./core.run
```

## Wabash services

To run the Wabash services,
```
# this requires the OpenCR connected
$ ./run_all.sh
```

To clean them up,
```
$ ./clean.sh
```

## Camera Streaming

To run camera streaming,
```
$ cp camera.run camera_left.run
$ nano camera_left.run
# modify IP to the IP address of the node on which core.run runs and NAME
# DEVICE should be the path of the camera (which might be mapped to /dev/wabash_cam_*
$ ./camera_left.run
```

To run web video service,
```
$ nano video_server.run
# modify IP to the IP address of the node on which core.run runs
$./video_server.run
```

On remote (base station) open a browser (Chrome is recommended) and type
```
# replace IP with the actual IP address of the core node
http://IP:8080
```

Finally, click the camera you want to open.
