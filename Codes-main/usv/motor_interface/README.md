# Arduino Mega for Interfacing with Motor drivers

An arduino mega is used to bridge interfaces between SmartDriveDuo-30/60 and Jetson.

## Commands
- `th [l, r] INT`: change direction/velocity of left/right thrusters
- `an [l, r] INT`: change direction/velocity of left/right anchors
- `sa [l, r] INT`: change direction/velocity of left/right sampler
__NOTE: only `r` is used__
- `cll [on, off]`: change on/off of left clutch
- `clr [on, off]`: change on/off of right clutch
- `cl [on, off]`: change on/off of sampler clutch
- `init`: send the init signal to the motor drivers

## Clutch Pins
```
uint8_t clutch_sampler_pin = 2;
uint8_t clutch_left_pin = 3;
uint8_t clutch_right_pin = 4;
```