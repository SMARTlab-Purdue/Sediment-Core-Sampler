# Jetson Nano for USV

## Camera setup

```
# on sudo
$ cat /etc/udev/rules.d/10-wabash.rules 
SUBSYSTEM=="tty" ATTRS{idProduct}=="0042", ATTRS{idVendor}=="2341", ATTRS{manufacturer}=="Arduino*", SYMLINK+="wabash_motor_driver"
SUBSYSTEM=="tty" ATTRS{idProduct}=="5740", ATTRS{idVendor}=="0483", ATTRS{manufacturer}=="ROBOTIS", SYMLINK+="wabash_sampler"
SUBSYSTEM=="tty" ATTRS{idProduct}=="0043", ATTRS{idVendor}=="2341", SYMLINK+="wabash_gps"
SUBSYSTEM=="tty" ATTRS{idProduct}=="xxxx", ATTRS{idVendor}=="yyyy", SYMLINK+="wabash_pressure"

SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", ATTRS{idVendor}=="05a3", ATTRS{idProduct}=="9420", ATTRS{devpath}=="2.1", SYMLINK+="wabash_cam_left"
SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", ATTRS{idVendor}=="05a3", ATTRS{idProduct}=="9420", ATTRS{devpath}=="2.2", SYMLINK+="wabash_cam_right"
SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", ATTRS{idVendor}=="05a3", ATTRS{idProduct}=="9422", SYMLINK+="wabash_cam_fisheye_other"
SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", ATTRS{idVendor}=="32e4", ATTRS{idProduct}=="9230", SYMLINK+="wabash_cam_fisheye"
SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", ATTRS{idVendor}=="534d", ATTRS{idProduct}=="2109", SYMLINK+="wabash_cam_sampler"
(add more camera here)

sudo -i udevadm control --reload-rules 
sudo -i udevadm trigger 
ls -lh /dev/wabash*
```
