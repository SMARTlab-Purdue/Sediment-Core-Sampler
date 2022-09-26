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

#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif    

#define BAUDRATE  57600
#define DXL_ID_1  1
#define DXL_ID_2  2
#define DXL_ID_3  3


DynamixelWorkbench dxl_wb;


float DXL_ID_1_Present_Velocity = 0;
int32_t DXL_ID_1_Present_Position, DXL_ID_2_Present_Position, DXL_ID_3_Present_Position;


ros::NodeHandle nh;

/*******************************************/
// Publisher Parts
/*******************************************/
//Pub_MOTORS
std_msgs::Float32 dx_1_read_current_velocity_msg;
ros::Publisher pub_dx_1_velocity_read("opencr/DX_1/read_velocity", &dx_1_read_current_velocity_msg);
std_msgs::Int32 dx_1_read_current_position_msg;
ros::Publisher pub_dx_1_position_read("opencr/DX_1/read_position", &dx_1_read_current_position_msg);
std_msgs::Int32 dx_2_read_current_position_msg;
ros::Publisher pub_dx_2_position_read("opencr/DX_2/read_position", &dx_2_read_current_position_msg);
std_msgs::Int32 dx_3_read_current_position_msg;
ros::Publisher pub_dx_3_position_read("opencr/DX_3/read_position", &dx_3_read_current_position_msg);

//Pub_IMU
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("opencr/imu", &imu_msg);
geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;
cIMU imu;

//Pub_VOLTAGE
std_msgs::Float32 voltage_msg;
ros::Publisher pub_voltage("opencr/voltage", &voltage_msg);

//Pub_BUTTON
std_msgs::Byte button_msg;
ros::Publisher pub_button("opencr/button", &button_msg);


/*******************************************/
// Subscribber Parts
/*******************************************/

void dx1_write_speed( const std_msgs::Int32& dx1_write_speed_msg) {
  int32_t dx1_write_speed = dx1_write_speed_msg.data;
  dxl_wb.goalSpeed(DXL_ID_1,dx1_write_speed);
}
ros::Subscriber<std_msgs::Int32> sub_DX1_write_speed("opencr/DX_1/write_speed", dx1_write_speed);

void dx2_write_position( const std_msgs::Int32& dx2_write_position_msg) {
  int32_t dx2_write_position = dx2_write_position_msg.data;
  dxl_wb.goalPosition(DXL_ID_2, dx2_write_position);
}
ros::Subscriber<std_msgs::Int32> sub_DX2_write_speed("opencr/DX_2/write_position", dx2_write_position);

void dx3_write_position( const std_msgs::Int32& dx3_write_position_msg) {
  int32_t dx3_write_position = dx3_write_position_msg.data;
  dxl_wb.goalPosition(DXL_ID_3, dx3_write_position);
}
ros::Subscriber<std_msgs::Int32> sub_DX3_write_speed("opencr/DX_3/write_position", dx3_write_position);


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
ros::Subscriber<std_msgs::Byte> sub_solenoid_control("opencr/solenoid", solenoid_control );


void setup()
{  
  uint16_t model_number = 0;
  
  //BDPIN_GPIO_1  -> limit_SW 1 //read
  //BDPIN_GPIO_2  -> solenoid 2 //read
  pinMode(BDPIN_GPIO_1, INPUT);
  pinMode(BDPIN_GPIO_2, OUTPUT);
  
  // Dynmixel Motor setting
  /****************************************/
  /***********  DYNMIXEL MOTOR  ***********/
  /****************************************/
  dxl_wb.init(DEVICE_NAME, BAUDRATE);

  dxl_wb.ping(DXL_ID_1, &model_number);
  dxl_wb.ping(DXL_ID_2, &model_number);
  dxl_wb.ping(DXL_ID_3, &model_number);

  dxl_wb.wheelMode(DXL_ID_1);  
  dxl_wb.jointMode(DXL_ID_2);
  dxl_wb.jointMode(DXL_ID_3);

 
  /****************************************/
  // ROS 
  /****************************************/
  nh.initNode();
  /****************************************/
  // Publisher Parts
  /****************************************/
  nh.advertise(pub_voltage);
  nh.advertise(imu_pub);
  tfbroadcaster.init(nh);
  //buttons
  nh.advertise(pub_button);
  //Motors
  nh.advertise(pub_dx_1_velocity_read);  
  nh.advertise(pub_dx_1_position_read);
  nh.advertise(pub_dx_2_position_read);
  nh.advertise(pub_dx_3_position_read);

  
  /****************************************/
  // Subscriber Parts
  /****************************************/
  nh.subscribe(sub_DX1_write_speed);
  nh.subscribe(sub_DX2_write_speed);
  nh.subscribe(sub_DX3_write_speed);
  nh.subscribe(sub_solenoid_control);

  imu.begin();

}

void loop()
{
  uint8_t btn_reading = 0;
  static uint32_t pre_time;

  imu.update();
  
  if (digitalRead(BDPIN_GPIO_1) == HIGH)
  {
    btn_reading |= 0x01;
  }
  

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

    imu_msg.orientation.w = imu.quat[0];
    imu_msg.orientation.x = imu.quat[1];
    imu_msg.orientation.y = imu.quat[2];
    imu_msg.orientation.z = imu.quat[3];

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
    
    // Voltage sensor
    voltage_msg.data = getPowerInVoltage();
    pub_voltage.publish(&voltage_msg);

    //Button 
    button_msg.data = btn_reading;
    pub_button.publish(&button_msg);
    
    // Current Positions
    dxl_wb.getPresentPositionData(DXL_ID_1, &DXL_ID_1_Present_Position);  
    dx_1_read_current_position_msg.data=DXL_ID_1_Present_Position;
  
    dxl_wb.getPresentPositionData(DXL_ID_2, &DXL_ID_2_Present_Position);  
    dx_2_read_current_position_msg.data=DXL_ID_2_Present_Position;
  
    dxl_wb.getPresentPositionData(DXL_ID_3, &DXL_ID_3_Present_Position);  
    dx_3_read_current_position_msg.data=DXL_ID_3_Present_Position;
  
    // Current Velocity
    dxl_wb.getVelocity(DXL_ID_1, &DXL_ID_1_Present_Velocity);
    dx_1_read_current_velocity_msg.data=DXL_ID_1_Present_Velocity;

    //Motors
    pub_dx_1_position_read.publish(&dx_1_read_current_position_msg);
    pub_dx_2_position_read.publish(&dx_2_read_current_position_msg);
    pub_dx_3_position_read.publish(&dx_3_read_current_position_msg);
    pub_dx_1_velocity_read.publish(&dx_1_read_current_velocity_msg);

    
    pre_time = millis();    
  }

  nh.spinOnce();
}
