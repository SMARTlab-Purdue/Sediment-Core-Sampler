# OpenCR flashing on Jetson

Use the opencr_ld binary to flash firmware to OpenCR board

```
$ export OPENCR_PORT=/dev/ttyACM0
$ binary_arm64/opencr_ld ${OPENCR_PORT} 115200 /PATH/TO/FIRMWARE.bin 1
opencr_ld ver 1.0.4
opencr_ld_main 
>>
file name : /home/smartlab/sampler/firmware.ino.bin 
file size : 175 KB
Open port OK
Clear Buffer Start
Clear Buffer End
Board Name : OpenCR R1.0
Board Ver  : 0x17020800
Board Rev  : 0x00000000
>>
flash_erase : 0 : 1.079000 sec
flash_write : 0 : 0.995000 sec 
CRC OK 10C7CE4 10C7CE4 0.006000 sec
[OK] Download 
jump_to_fw 
jump finished
```
