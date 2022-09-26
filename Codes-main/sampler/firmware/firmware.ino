/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */


#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Byte.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <IMU.h>

#include <SPI.h>

//#include <Wire.h>
//#include <SparkFun_MS5803_I2C.h> // Click here to get the library: http://librarymanager/All#SparkFun_MS5803-14BA

//MS5803 sensor(ADDRESS_HIGH);

#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif

#define BAUDRATE  57600
#define STALL_CURRENT 4100 // Stall current mA
#define DXL_ID_1  1
#define DXL_ID_2  2

#define SPI_MAX_speed 500000
SPISettings set1(SPI_MAX_speed, MSBFIRST, SPI_MODE0);
#define SDADtransmission 0xA6
#define SDADstatus 0xF5

#define ENCODER_PIN 8
#define RESOLUTION 40

DynamixelWorkbench dxl_wb;


float DXL_ID_1_Present_Velocity = 0;
int32_t DXL_ID_1_Present_Position, DXL_ID_2_Present_Position;
float DXL_ID_1_Current, DXL_ID_2_Current;

int16_t potentiometer_linear_pin = A0;
int16_t potentiometer_rotary_pin = A2;

// Magnetic encoder linear CS pin
uint8_t encoder_linear_cs=9;
// Magnetic encoder rotary CS pin
uint8_t encoder_rotary_cs=10;

#define STRING_BUF_NUM 64
String pressure_reading[STRING_BUF_NUM];
// bool is_new_upper_pressure = false;
// bool is_new_lower_pressure = false;

//Reset pin
int Reset = 12;

// Encoder counter
unsigned long gulCount = 0;
unsigned long gulStart_Timer = 0;

// Indicators for upper and lower limit switches
uint8_t is_upper_limit_pressed = 0;
uint8_t is_lower_limit_pressed = 0;

// When upper limit switch is pressed
// stage 0: Move linear motor down at full speed for a second
// stage 1: Stop the linear motor
uint32_t emergency_upper_limit_stage1;

// When lower limit switch is pressed
// stage 0: Stage rotary motor to center and move linear motor up for a second
// stage 1: Stop the linear motor
// stage 2: Move linear motor up (to the top)
uint32_t emergency_lower_limit_stage1;
uint32_t emergency_lower_limit_stage2;


ros::NodeHandle nh;

/*******************************************/
// Publisher Parts
/*******************************************/
//Pub_MOTORS
std_msgs::Float32 dx_1_read_current_velocity_msg;
ros::Publisher pub_dx_1_velocity_read("sampler/motor/linear/read_velocity", &dx_1_read_current_velocity_msg);
std_msgs::Int32 dx_1_read_current_position_msg;
ros::Publisher pub_dx_1_position_read("sampler/motor/linear/read_position", &dx_1_read_current_position_msg);
std_msgs::Float32 dx_1_read_current_msg;
ros::Publisher pub_dx_1_current_read("sampler/motor/linear/current", &dx_1_read_current_msg);
std_msgs::Int32 dx_2_read_current_position_msg;
ros::Publisher pub_dx_2_position_read("sampler/motor/rotary/read_position", &dx_2_read_current_position_msg);
std_msgs::Float32 dx_2_read_current_msg;
ros::Publisher pub_dx_2_current_read("sampler/motor/rotary/current", &dx_2_read_current_msg);


//Pub_IMU
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("sampler/imu", &imu_msg);
geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;
cIMU imu;

//Pub_MAGNETIC_ENCODER_LINEAR
std_msgs::Int32 mag_encoder_linear_msg;
ros::Publisher pub_mag_encoder_linear("sampler/magnetic/linear/abs_position", &mag_encoder_linear_msg);

//Pub_MAGNETIC_ENCODER_ROTARY
std_msgs::Int32 mag_encoder_rotary_msg;
ros::Publisher pub_mag_encoder_rotary("sampler/magnetic/rotary/abs_position", &mag_encoder_rotary_msg);

//Pub_POTENTIOMETER_LINEAR
std_msgs::Int32 pot_linear_msg;
ros::Publisher pub_pot_linear("sampler/potentiometer/linear", &pot_linear_msg);

//Pub_POTENTIOMETER_ROTARY
std_msgs::Int32 pot_rotary_msg;
ros::Publisher pub_pot_rotary("sampler/potentiometer/rotary", &pot_rotary_msg);

//Pub_PRESSURE
// std_msgs::Float32 pressure_upper_msg;
// ros::Publisher pub_upper_pressure_sensor("sampler/pressure/upper/pressure", &pressure_upper_msg);
// std_msgs::Float32 pressure_lower_msg;
// ros::Publisher pub_lower_pressure_sensor("sampler/pressure/lower/pressure", &pressure_lower_msg);

//Pub_TEMPERATURE_FROM_PRESSURE_SENSOR
// std_msgs::Float32 temperature_upper_msg;
// ros::Publisher pub_upper_temperature_sensor("sampler/pressure/upper/temp", &temperature_upper_msg);
// std_msgs::Float32 temperature_lower_msg;
// ros::Publisher pub_lower_temperature_sensor("sampler/pressure/lower/temp", &temperature_lower_msg);

//Pub_VOLTAGE
std_msgs::Float32 voltage_msg;
ros::Publisher pub_voltage("sampler/voltage", &voltage_msg);

//Pub_LIMIT_UPPER
std_msgs::Byte limit_upper_msg;
ros::Publisher pub_limit_upper("sampler/switch/upper", &limit_upper_msg);

//Pub_LIMIT_LOWER
std_msgs::Byte limit_lower_msg;
ros::Publisher pub_limit_lower("sampler/switch/lower", &limit_lower_msg);

//Pub_Encoder_counter
std_msgs::Int32 encoder_counter_msg;
ros::Publisher pub_encoder_counter("sampler/encoder/linear/counter", &encoder_counter_msg);


/*******************************************/
// Subscribber Parts
/*******************************************/

void dx1_write_speed( const std_msgs::Int32& dx1_write_speed_msg) {
  int32_t dx1_write_speed = dx1_write_speed_msg.data;
  // float32_t dx1_write_speed = dx1_write_speed_msg.data; // Unit is m/s
  dxl_wb.goalSpeed(DXL_ID_1,dx1_write_speed);
  // dxl_wb.goalVelocity(DXL_ID_1, dx1_write_speed);
}
ros::Subscriber<std_msgs::Int32> sub_DX1_write_speed("sampler/motor/linear/write_speed", dx1_write_speed);

void dx2_write_position( const std_msgs::Int32& dx2_write_position_msg) {
  int32_t dx2_write_position = dx2_write_position_msg.data;
  dxl_wb.goalPosition(DXL_ID_2, dx2_write_position);
}
ros::Subscriber<std_msgs::Int32> sub_DX2_write_speed("sampler/motor/rotary/write_position", dx2_write_position);

void solenoid_control( const std_msgs::Byte& solenoid_msg) {
  if (solenoid_msg.data & 1)
  {
    digitalWrite(BDPIN_GPIO_2, HIGH);
  }
  else
  {
    digitalWrite(BDPIN_GPIO_2, LOW);
  }
}
ros::Subscriber<std_msgs::Byte> sub_solenoid_control("sampler/solenoid", solenoid_control );

bool do_calibrate = false;
void imu_calibration( const std_msgs::Byte& not_used) {
  do_calibrate = true;
}
ros::Subscriber<std_msgs::Byte> sub_imu_calibration("sampler/imu/do_calibration", imu_calibration );

void reset(const std_msgs::Byte& resetbutton_msg){
  digitalWrite(Reset, LOW);
}
ros::Subscriber<std_msgs::Byte> sub_reset("sampler/reset", reset);


void setup()
{
  digitalWrite(Reset, HIGH);
  
  // Serial for Arduino micro with pressure sensors
  // Serial1.begin(9600);

  nh.initNode();

//  Serial.begin(115200);
//  while(!Serial);
//  const char* log;
//  bool result = false;
  uint16_t model_number = 0;

  //BDPIN_GPIO_1  -> limit_SW Upper //read
  //BDPIN_GPIO_3  -> limit_SW Lower //read
  //BDPIN_GPIO_2  -> solenoid 2 //read
  pinMode(BDPIN_GPIO_1, INPUT_PULLDOWN);
  pinMode(BDPIN_GPIO_3, INPUT_PULLDOWN);
  pinMode(BDPIN_GPIO_2, OUTPUT);

  pinMode(encoder_linear_cs, OUTPUT);
  pinMode(encoder_rotary_cs, OUTPUT);

  pinMode(8, INPUT_PULLDOWN);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);

  // Dynmixel Motor setting
  /****************************************/
  /***********  DYNMIXEL MOTOR  ***********/
  /****************************************/
  dxl_wb.init(DEVICE_NAME, BAUDRATE);

  dxl_wb.ping(DXL_ID_1, &model_number);
//  Serial.println("helloworld");
  dxl_wb.ping(DXL_ID_2, &model_number);
//  if (result == false) {
//     Serial.println(log);
//     Serial.println("Failed to init");
//  } else
//  {
//    Serial.println("Succeeded to ping");
//    Serial.print("id : ");
//    Serial.print(DXL_ID_2);
//    Serial.print(" model_number : ");
//    Serial.println(model_number);
//  }
//  dxl_wb.ping(DXL_ID_3, &model_number);

  dxl_wb.wheelMode(DXL_ID_1);
//  Serial.println("about to change mode");
  dxl_wb.jointMode(DXL_ID_2);
//  Serial.println("changed mode");
//  if (result == false)
//  {
//    Serial.println(log);
//  }
//  else
//  {
//    Serial.println(log);
//  }
//  dxl_wb.jointMode(DXL_ID_3);

  //attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), COUNT, CHANGE);
  attachInterrupt(4, COUNT, CHANGE);


  /****************************************/
  // Publisher Parts
  /****************************************/
  nh.advertise(pub_voltage);
  nh.advertise(imu_pub);
  tfbroadcaster.init(nh);
  //buttons
  nh.advertise(pub_limit_upper);
  nh.advertise(pub_limit_lower);
  //Motors
  nh.advertise(pub_dx_1_velocity_read);
  nh.advertise(pub_dx_1_position_read);
  nh.advertise(pub_dx_2_position_read);
  nh.advertise(pub_dx_1_current_read);
  nh.advertise(pub_dx_2_current_read);
  //Magnetic encoders
  nh.advertise(pub_mag_encoder_linear);
  nh.advertise(pub_mag_encoder_rotary);
  //Potentiometers
  nh.advertise(pub_pot_linear);
  nh.advertise(pub_pot_rotary);
  //Pressure and Temperature
//  nh.advertise(pub_upper_pressure_sensor);
//  nh.advertise(pub_lower_pressure_sensor);
//  nh.advertise(pub_upper_temperature_sensor);
//  nh.advertise(pub_lower_temperature_sensor);
  nh.advertise(pub_encoder_counter);

  /****************************************/
  // Subscriber Parts
  /****************************************/
  nh.subscribe(sub_DX1_write_speed);
  nh.subscribe(sub_DX2_write_speed);
  nh.subscribe(sub_solenoid_control);
  nh.subscribe(sub_imu_calibration);
  nh.subscribe(sub_reset);
  
  imu.begin();
  delay(1000);


  /****************************************/
  // Reset OpenCR board
  /****************************************/
  //digitalWrite(Reset, HIGH);
  //delay(200); 
  pinMode(Reset, OUTPUT);     
  delay(200);

  // Initialize emergency stages
  emergency_upper_limit_stage1 = millis();
  emergency_lower_limit_stage1 = millis();
  emergency_lower_limit_stage2 = millis();
}

uint8_t val1, val2, val3;
// MSB reg: 165
// LSB reg: 0
// 19
//
// MSB bit 18
// LSB 0
// ZEROs 5
// total = 24 / 8 = 3 bytes
//
// MPC reg: 5
// Thus, we measure absolute value of 3 bytes by SDAD command.
// 23 22 21 20 19 .... 5   4 3 2 1 0
// MSB                 LSB   ZEROS
uint32_t readMagneticEncoderData(uint8_t ss)
{
  uint8_t interval = 20;
  SPI.beginTransaction(set1);
  digitalWrite(ss, LOW);
  delay(interval);
  SPI.transfer(SDADtransmission); // send command byte 151
  delay(interval);

  val1 = SPI.transfer(0x00);
  delay(interval);
  val2 = SPI.transfer(0x00);
  delay(interval);
  val3 = SPI.transfer(0x00);
  delay(interval);
  uint32_t value = (val1 << 16) | (val2 << 8) | (val3);
  value >>= 5;
  digitalWrite(ss, HIGH);
  SPI.endTransaction();
  return value;
}

void COUNT(void)
{
  gulCount++;
}

unsigned long readEncoder() {
  //unsigned long ulRPM = 0;
  //unsigned long ulTimeDif = 0;

  //detachInterrupt(digitalPinToInterrupt(ENCODER_PIN));
  detachInterrupt(4);

  //ulTimeDif = millis() - gulStart_Timer;
  //ulRPM = 60000*gulCount;
  //ulRPM = ulRPM/ulTimeDif;
  //ulRPM = ulRPM/RESOLUTION;
  //usRPM = ((60*1000*gulCount)/ulTimeDif)/RESOLUTION;
  
  unsigned long e = gulCount;
  gulCount = 0;    
  //gulStart_Timer = millis();
  //attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), COUNT, CHANGE);
  attachInterrupt(4, COUNT, CHANGE);
  return e;
}

void split(String data, char separator, String* temp)
{
  int cnt = 0;
  int get_index = 0;

  String copy = data;

  while(true)
  {
    get_index = copy.indexOf(separator);

    if(-1 != get_index)
    {
      temp[cnt] = copy.substring(0, get_index);

      copy = copy.substring(get_index + 1);
    }
    else
    {
      temp[cnt] = copy.substring(0, copy.length());
      break;
    }
    ++cnt;
  }
}

void loop()
{
  const char* log;
  bool result = false;

  int32_t current_value = 0;
  static uint32_t pre_time;
  static uint32_t current_time;

  if (is_upper_limit_pressed == 1)
  {
    if (millis() > emergency_upper_limit_stage1)
    {
      dxl_wb.goalSpeed(DXL_ID_1,0);
      // End of emergency
      is_upper_limit_pressed = 0;
    }
  }
  else if (digitalRead(BDPIN_GPIO_1) == HIGH)
  {
    is_upper_limit_pressed |= 0x01;
    // Go down for 5 mm
    // TODO: need to know how to go 5 mm
    // Limit switches
    // limit_upper_msg.header.stamp = nh.now();
    limit_upper_msg.data = is_upper_limit_pressed;
    pub_limit_upper.publish(&limit_upper_msg);
    //dxl_wb.goalPosition(DXL_ID_2,2048);
    //dxl_wb.goalSpeed(DXL_ID_1,1022); //CCW 0 - 1023 -DOWN
    dxl_wb.goalSpeed(DXL_ID_1,2045);
    emergency_upper_limit_stage1 = millis() + 1000; // 1 second delay
  }

  if (is_lower_limit_pressed == 1)
  {
    if (millis() > emergency_lower_limit_stage2)
    {
      //dxl_wb.goalSpeed(DXL_ID_1, 2045);//CW 1024 - 2047 -UP
      dxl_wb.goalSpeed(DXL_ID_1, 1022);
      // End of emergency
      is_lower_limit_pressed = 0;
    }
    else if (millis() > emergency_lower_limit_stage1)
    {
      dxl_wb.goalSpeed(DXL_ID_1, 0);
    }
  }
  else if (digitalRead(BDPIN_GPIO_3) == HIGH)
  {
    is_lower_limit_pressed |= 0x01;
    // limit_lower_msg.header.stamp = nh.now();
    limit_lower_msg.data = is_lower_limit_pressed;
    pub_limit_lower.publish(&limit_lower_msg);
    current_time = millis();
    // Junhan requested; wait for 5 seconds and pull the sampler up
    emergency_lower_limit_stage1 = millis() + 1000; // 1 second delay
    emergency_lower_limit_stage2 = millis() + 6000; // 5 seconds delay from stage 2
    dxl_wb.goalPosition(DXL_ID_2,2048);
    dxl_wb.goalSpeed(DXL_ID_1, 500);
  }

  // Calibration is performed only when there is no emergency
  if (is_upper_limit_pressed == 0 && is_lower_limit_pressed == 0)
  {
    // Calibrate IMU sensor when triggered
    if (do_calibrate) {
      // stop the motors for safety
      dxl_wb.goalSpeed(DXL_ID_1, 0);
      dxl_wb.goalPosition(DXL_ID_2,2048);
      imu.SEN.acc_cali_start();
      while( imu.SEN.acc_cali_get_done() == false )
      {
        imu.update();
      }
      do_calibrate = false;
    }
  }

  imu.update();

  // Update pressure sensor readings
  // if (Serial1.available()) {
  //   String read_string = Serial1.readStringUntil('\n');
  //   read_string.trim();
  //   split(read_string, ' ', pressure_reading);
  //   // up 996.60 ut 25.83 lp 997.00 lt 25.67
  //   if (pressure_reading[0] == "up") {
  //     pressure_upper_msg.data = pressure_reading[1].toFloat();
  //     is_new_upper_pressure = true;
  //   }
  //   if (pressure_reading[2] == "ut") {
  //     temperature_upper_msg.data = pressure_reading[3].toFloat();
  //   }
  //   if (pressure_reading[4] == "lp") {
  //     pressure_lower_msg.data = pressure_reading[5].toFloat();
  //     is_new_lower_pressure = true;
  //   }
  //   if (pressure_reading[6] == "lt") {
  //     temperature_lower_msg.data = pressure_reading[7].toFloat();
  //   }
  // }

  if (millis()-pre_time >= 10)
  {
    //IMU sensor
    imu_msg.header.stamp    = nh.now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.angular_velocity.x = imu.gyroData[0];
    imu_msg.angular_velocity.y = imu.gyroData[1];
    imu_msg.angular_velocity.z = imu.gyroData[2];
    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.02;

    imu_msg.linear_acceleration.x = imu.accData[0];
    imu_msg.linear_acceleration.y = imu.accData[1];
    imu_msg.linear_acceleration.z = imu.accData[2];
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.orientation.w = 0.;
    imu_msg.orientation.x = imu.rpy[0];
    imu_msg.orientation.y = imu.rpy[1];
    imu_msg.orientation.z = imu.rpy[2];

    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 0.0025;

    imu_pub.publish(&imu_msg);

    tfs_msg.header.stamp    = nh.now();
    tfs_msg.header.frame_id = "base_link";
    tfs_msg.child_frame_id  = "imu_link";
    tfs_msg.transform.rotation.w = imu.quat[0];
    tfs_msg.transform.rotation.x = imu.quat[1];
    tfs_msg.transform.rotation.y = imu.quat[2];
    tfs_msg.transform.rotation.z = imu.quat[3];

    tfs_msg.transform.translation.x = 0.0;
    tfs_msg.transform.translation.y = 0.0;
    tfs_msg.transform.translation.z = 0.0;

    tfbroadcaster.sendTransform(tfs_msg);

    // switches
    // limit_upper_msg.header.stamp = nh.now();
    limit_upper_msg.data = is_upper_limit_pressed;
    pub_limit_upper.publish(&limit_upper_msg);
    // limit_lower_msg.header.stamp = nh.now();
    limit_lower_msg.data = is_lower_limit_pressed;
    pub_limit_lower.publish(&limit_lower_msg);

    // Voltage sensor
    voltage_msg.data = getPowerInVoltage();
    pub_voltage.publish(&voltage_msg);

    // Current Positions
    dxl_wb.getPresentPositionData(DXL_ID_1, &DXL_ID_1_Present_Position);
    dx_1_read_current_position_msg.data=DXL_ID_1_Present_Position;

    dxl_wb.getPresentPositionData(DXL_ID_2, &DXL_ID_2_Present_Position);
    dx_2_read_current_position_msg.data=DXL_ID_2_Present_Position;

    // Current Velocity
    dxl_wb.getVelocity(DXL_ID_1, &DXL_ID_1_Present_Velocity);
    // dx_1_read_current_velocity_msg.header.stamp = nh.now();
    dx_1_read_current_velocity_msg.data = DXL_ID_1_Present_Velocity;
    pub_dx_1_velocity_read.publish(&dx_1_read_current_velocity_msg);

    //Motors
    pub_dx_1_position_read.publish(&dx_1_read_current_position_msg);
    pub_dx_2_position_read.publish(&dx_2_read_current_position_msg);

    // Currents
    result = dxl_wb.itemRead(DXL_ID_1, "Current", &current_value, &log);
    if (result == true) {
      DXL_ID_1_Current = (float)(4.5 * (current_value - 2048));
      // if (DXL_ID_1_Current >= STALL_CURRENT) {
      //   dxl_wb.goalSpeed(DXL_ID_1, 0);
      // }
      dx_1_read_current_msg.data = DXL_ID_1_Current;
      pub_dx_1_current_read.publish(&dx_1_read_current_msg);
    }
    result = dxl_wb.itemRead(DXL_ID_2, "Current", &current_value, &log);
    if (result == true) {
      DXL_ID_2_Current = (float)(4.5 * (current_value - 2048));
      // if (DXL_ID_2_Current >= STALL_CURRENT) {
      //   dxl_wb.goalPosition(DXL_ID_2, (int)DXL_ID_2_Present_Position);
      // }
      dx_2_read_current_msg.data = DXL_ID_2_Current;
      pub_dx_2_current_read.publish(&dx_2_read_current_msg);
    }

    // Magnetic encoders
    mag_encoder_linear_msg.data = readMagneticEncoderData(encoder_linear_cs);
    pub_mag_encoder_linear.publish(&mag_encoder_linear_msg);
    mag_encoder_rotary_msg.data = readMagneticEncoderData(encoder_rotary_cs);
    pub_mag_encoder_rotary.publish(&mag_encoder_rotary_msg);

    // if (is_new_upper_pressure) {
    //   pub_upper_pressure_sensor.publish(&pressure_upper_msg);
    //   pub_upper_temperature_sensor.publish(&temperature_upper_msg);
    //   is_new_upper_pressure = false;
    // }
    // if (is_new_lower_pressure) {
    //   pub_lower_pressure_sensor.publish(&pressure_lower_msg);
    //   pub_lower_temperature_sensor.publish(&temperature_lower_msg);
    //   is_new_lower_pressure = false;
    // }

    // Potentiometer
    pot_linear_msg.data = analogRead(potentiometer_linear_pin);
    pub_pot_linear.publish(&pot_linear_msg);
    pot_rotary_msg.data = analogRead(potentiometer_rotary_pin);
    pub_pot_rotary.publish(&pot_rotary_msg);

    encoder_counter_msg.data = readEncoder();
    pub_encoder_counter.publish(&encoder_counter_msg);

    pre_time = millis();
  }

  nh.spinOnce();
}
