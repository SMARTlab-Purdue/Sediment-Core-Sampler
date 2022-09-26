/*
 SPI bus demo using a Microchip MCP4162 digital potentiometer [http://bit.ly/iwDmnd]
 */
#include <SPI.h> // necessary library
int ss=10; // using digital pin 10 for SPI slave select
int ss2=9;
int del=5000; // used for various delays, ms
uint8_t val1, val2, val3, val4;
int interval=200;
#define SPI_MAX_speed 500000
SPISettings set1(SPI_MAX_speed, MSBFIRST, SPI_MODE0);
#define SDADtransmission 0xA6
#define SDADstatus 0xF5
void setup()
{
  Serial.begin(115200);
  pinMode(ss, OUTPUT); // we use this for SS pin (chip select CS)
  pinMode(ss2, OUTPUT);
  SPI.begin(); // wake up the SPI bus.
  SPI.setBitOrder(MSBFIRST);
  // ic MU150 requires data to be sent MSB (most significant byte) first
}
//void setValue(int value)
//{
//  SPI.beginTransaction(set1);
//  digitalWrite(ss, LOW);
//  delay(interval);
//
//  Serial.println(SPI.transfer(0x97)); // send command byte 151
//  delay(interval);
//  Serial.println(SPI.transfer(value)); // send value (0~255)  51
//  delay(interval);
//  digitalWrite(ss, HIGH);
//  SPI.endTransaction();
//
//  delay(1000);
//  
//  SPI.beginTransaction(set1);
//  digitalWrite(ss, LOW);
//  delay(interval);
//  Serial.println(SPI.transfer(0xAD)); // OP  173 read data
//  delay(interval);
//  Serial.println(SPI.transfer(0x00)); // Status   0 read status
//  delay(interval);
//  Serial.println(SPI.transfer(0x65)); // DATA  0   read data sending meaningless byte
//  delay(interval);
//
//  digitalWrite(ss, HIGH);
//  SPI.endTransaction();
//}
void readData(int ss)
{
  SPI.beginTransaction(set1);
  digitalWrite(ss, LOW);
  delay(interval);
  Serial.println(SPI.transfer(SDADtransmission)); // send command byte 151
  delay(interval);

  val1 = SPI.transfer(0x00);
  delay(interval);
  val2 = SPI.transfer(0x00);
  delay(interval);
  val3 = SPI.transfer(0x00);
  delay(interval);
  int value = (val1 << 16) | (val2 << 8) | (val3);
  value >>= 5;
  Serial.println(value);
  digitalWrite(ss, HIGH);
  SPI.endTransaction();
  
}
void loop()
{
  
  delay(500);
  Serial.println("=============================");
  Serial.print("Linear: ");
  readData(9);
  Serial.print("Rotary: ");
  readData(10);
//  for (int i = 112; i < 112+6; i++)
//  {
//    setValue(i);
//    delay(del);
//  }
//    Serial.println("=============================");
//    setValue(0x11);    //   165
//    delay(del);
//    Serial.println("=============================");
//
//    setValue(0x12);    //   0
//    delay(del);
//    Serial.println("=============================");
//
//    setValue(0x15);     //  19
//    delay(del);
//    Serial.println("=============================");
//    
//    setValue(0x0F);     //  5
//    delay(del);
//    Serial.println("=============================");
}
