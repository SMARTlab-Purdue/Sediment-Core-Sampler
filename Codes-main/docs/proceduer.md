# Set ups

0. Set up network
 - pass: SmartROS recognizable from base station

1. main(12 V) power on; MCU 1, 2, and 3 power on
 - pass: MCU 1, 2, and 3 power/fan on

2. Nano power on
 - pass: check the green LED on

3. Arduino Mega reset
 - pass: MCU 1, 2, and 3 no blinking.
 - troubleshooting: Repeat 3 until success.

4. (from base station) ssh into nano
```
$ ssh junhan@192.168.1.4
```
 - pass: login successfully
 - troubleshooting: 192.168.1.1 -> check if nano is connected to the network; Nano's name is `junahn-desktop`

5. Sampler Controller / solenoid power on
 - pass: run the command on nano
 ```
$ ls -lha /dev/wabash_sampler
lrwxrwxrwx 1 root root 7 Mar 13 22:40 /dev/wabash_sampler -> ttyACM1
```

6. (in ssh) ros service runs okay
 - pass: the result shows Active (running)
 - troubhelshooting: sudo -i service wabashcore restart
```
$ sudo -i service wabashcore status
```
# **CHECK** If the IP address of the device changed, you have to reset the IP address.
```
$ nano setup.sh
$ source setup.sh 
```
and
```
$ systemctl status wabashcore
$ nano wabash.sh
$ sudo service wabashcore restart
```
MY_IP is the IP of the computer you run docker containers
MASTER_IP is the IP where WabashCore ROS is running (e.g., Jetson Nano) 

7. (in ssh) run source setup.sh
must run before ./run.all.sh (even if IP is not changed

8. (in ssh) launch wabash services
```
$ cd /home/junhan/repo/Codes/docker
$ ./run_all.sh
docker ps
CONTAINER ID        IMAGE               COMMAND                  CREATED             STATUS              PORTS               NAMES
97eb41a8ddde        wabash_ros          "/ros_entrypoint.sh …"   3 minutes ago       Up 3 minutes                            wabash_cam_fisheye
fb143d1255b8        wabash_ros          "/ros_entrypoint.sh …"   3 minutes ago       Up 3 minutes                            wabash_video_server
c75b6b758a35        wabash_ros          "/ros_entrypoint.sh …"   16 minutes ago      Up 16 minutes                           wabash_motor_driver
60dec6e54943        wabash_ros          "/ros_entrypoint.sh …"   16 minutes ago      Up 16 minutes                           wabash_distance_traveled
3dc0acf10f40        wabash_ros          "/ros_entrypoint.sh …"   16 minutes ago      Up 16 minutes                           wabash_solenoid_controller
191ef87043a8        wabash_ros          "/ros_entrypoint.sh …"   16 minutes ago      Up 16 minutes                           wabash_rotary_controller
1329ad7e1d2f        wabash_ros          "/ros_entrypoint.sh …"   16 minutes ago      Up 16 minutes                           wabash_sampler_serial
+ PRESSURE_DEPTH!!!
+ GPS!!!!
$ rostopic list (you should see many topics there!)
```

9. run UI on the base station
```
$ ./ui.run 192.168.1.3
$ cd /storage/
$ python3 ui.py
 - pass: you see the UI
 - troubleshooting: you may need to make the terminal full size
```

10. (in ssh) record the experiment
```
$ ./record.sh
Ctrl + C to stop
$ ls -lha 
2021-03-13-23-46-43.bag .....
```
11. (in base station) open browser (i.e. Chrome) to monitor USV via fisheye camera (use Jetson IP)

192.168.1.6:8080


12-a. (in another ssh) send out the USV manually
```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
press 'q' button to increase the speed
press 'z' button to decrease the speed
speed: 1 turn: 2 - max. performance of the thruster



12-b. (in another ssh) send out the USV in auto mode
```
$ cd /home/junhan/repo/Codes/docker/usv_autopilot
$ lat=LATITUDE lon=LONGITUDE ./auto.sh
Setting the autopilot to a coordinate: LATITUDE, LONGITUDE
+ rostopic pub -1 /usv/autopilot/cmd/engage std_msgs/Bool True
publishing and latching message for 3.0 seconds
+ rostopic pub -1 /usv/autopilot/destination sensor_msgs/NavSatFix '{latitude: LATUTUDE, longitude: LONGITUDE}'
publishing and latching message for 3.0 seconds
Done
+ set +x
```

11-c. killing autopilot in case of emergency
```
$ cd /home/junhan/repo/Codes/docker/usv_autopilot
$ ./kill.sh
Attempting to kill autopilot...
+ rostopic pub -1 /usv/autopilot/cmd/kill std_msgs/Bool True
publishing and latching message for 3.0 seconds
+ set +x
```




# How to Reset OpenCR board
```
$ rostopic pub -1 /sampler/reset std_msgs/Byte 0x00
```

