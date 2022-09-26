#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif

#define BAUDRATE  57600
#define DXL_ID  1

DynamixelWorkbench dxl_wb;

void setup()
{
  pinMode(BDPIN_GPIO_1, INPUT_PULLDOWN);
  pinMode(BDPIN_GPIO_3, INPUT_PULLDOWN);

  Serial.begin(115200);
  while(!Serial); // Wait for Opening Serial Monitor

  const char *log;
  bool result = false;

  uint8_t dxl_id = DXL_ID;
  uint16_t model_number = 0;

  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to init");
  }
  else
  {
    Serial.print("Succeeded to init : ");
    Serial.println(BAUDRATE);
  }

  result = dxl_wb.ping(dxl_id, &model_number, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to ping");
  }
  else
  {
    Serial.println("Succeeded to ping");
    Serial.print("id : ");
    Serial.print(dxl_id);
    Serial.print(" model_number : ");
    Serial.println(model_number);
  }
  dxl_wb.wheelMode(DXL_ID);
}

bool result = false;
int32_t vel = 0;
float vel2 = 0.f;
int32_t en_vel = 0;

void loop()
{
  if (digitalRead(BDPIN_GPIO_1) == HIGH)
  {
    dxl_wb.goalSpeed(DXL_ID, 0);
  } else if (digitalRead(BDPIN_GPIO_3) == HIGH)
  {
    dxl_wb.goalSpeed(DXL_ID, 0);
  }

  const char *log = NULL;

  if (Serial.available()) {
    String read_string = Serial.readStringUntil('\n');
    vel = read_string.toInt();
    result = dxl_wb.goalVelocity(DXL_ID, vel, &log);
    if (result == false) {
      Serial.println(log);
    }
    delay(1000);
    dxl_wb.goalVelocity(DXL_ID, 0, &log);
  }
  result = dxl_wb.getVelocity(DXL_ID, &vel2, &log);

  en_vel = dxl_wb.convertVelocity2Value(DXL_ID, vel2);
  if (result == false) {
    Serial.println(log);
  } else {
    Serial.print("Current velocity: ");
    Serial.println(vel2);
    Serial.print("Converted into int: ");
    Serial.println(en_vel);
  }
  delay(50);
}
