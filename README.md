# Uncrewed Remote Underwater Robotic Sediment Core Sampler
<p align="center">
<img src="/media/Unmanned_Sediment_Sampling_System_main.jpg" width="500" >
<img src="/media/USV_component_new-1.png" width="500" >
<img src="/media/Sediment_sampling_procedure-1.png" width="1000" >
</p>

Sediment has a significant impact on social, economic, and environmental systems. With the need for an effective sediment management and monitoring systems growing more important, a method for precisely and reproducibly obtaining sediment samples that represent the actual environment is essential for water resource management and researchers across aquatic domains (such as lakes, rivers, reservoirs, mine drainage ponds, and wastewater lagoons). To meet this need, robotic approaches for sediment sampling have been introduced. However, they are not tailored to a sediment sampling method and do not focus on the quality of the sediment sample. In this paper, we introduce an uncrewed sediment sampling system based on the unmanned surface vehicle (USV) and underwater sediment sampler (USS) to collect sediment samples and with the ability to comprehensively study sample quality from surface-water environments. The main objective of the USV is to carry the USS to the desired sampling area and then maintain its position while launching the USS to the bottom of the body of water and sampling the sediment. The core sampling method was adopted for the USS to extend the use of the sampled sediment by minimizing the disturbance while sampling. We integrated highly flexible sediment sampling patterns and self-adaptability mechanisms to investigate the samples’ reliability and quality, and tested the effectiveness of the flexible mechanisms. Through extensive experiments, we evaluated and validated the performance of the sediment sampling system in both lab and real environments and optimized the operation of the system as a whole.

# Component List and Cost
| USV Conponent                             | Price (USD)   |
| ---------------------------------------------- | --------- |
| Nvidia Jetson Nano                             | $109.00   |
| Cytron Thruster motor driver SmartDriveDuo-30  | $150.00   |
| Two Minn kota Thrusters Model: 1352230         | $220.00   |
| Expandacraft 12 foot outrigger kit for 2 sides | $1,054.00 |
| Two tsiny DC motors for winches TS-30GZ6287    | $58.00    |
| BADLAND 2500 Lbs Utility Electric Winch        | $70.00    |
| Sparkfun Compass - HMC6352                     | $17.00    |
| Sparkfun GPS - EM506                           | $40.00    |
| USB camera ELP-USBFHD01M-L21                   | $46.00    |
| Miscellaneous                   | ~$115.00    |
| **USV Cost**                          | **$2,900.00**   |

| USS Component                                          | Price (USD) |
| ------------------------------------------------------ | ----------- |
| Two Sparkfun Pressure sensors MS5803-14BA              | $20.00      |
| testco Solenoid - 100 STA                              | $27.00      |
| Two Sparkfun Potentiometers COM-09119                  | $10.00      |
| Lipo battery 3S1P                                      | $64.00      |
| Battery volt/current sensor - Pixhawk Power Module BEC | $14.00      |
| 8'' ePlastics acrylic tube                             | $315.00     |
| Two Bluerobotics 8'' Flange WTE8-M-FLANGE-SEAL-R1-RP   | $170.00     |
| 8'' Bluerobotics cover WTE8-M-END-CAP-R1-RP            | $42.00      |
| Two Rubber bellow B01IH8JKJI                           | $40.00      |
| 3 Linear thearded rods 3313N19                         | $53.00      |
| 4 550ml syringes KIK-PLA-SYR-011                       | $56.00      |
| PVC pipes                                              | $15.00      |
| Miscellaneous                   | ~$154.00    |
| **USS Total Cost**                                         | **$1,800.00**   |
| **Platform Total Cost**                                    | **$4,700.00**   |

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

