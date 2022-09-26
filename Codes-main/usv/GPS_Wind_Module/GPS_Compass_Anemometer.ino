#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Wire.h> 
#include "TimerOne.h" // Timer Interrupt set to 2 second for read sensors
#include <math.h>


// GPS setup //
// Define which pins you will use on the Arduino to communicate with your 
// GPS. In this case, the GPS module's TX pin will connect to the 
// Arduino's RXPIN which is pin 3.
#define RXPIN 4
#define TXPIN 3
//Set this value equal to the baud rate of your GPS
#define GPSBAUD 4800
float lat=0.;
float lng=0.;




// Create an instance of the TinyGPS object
TinyGPS gps;

// Initialize the NewSoftSerial library to the pins you defined above
SoftwareSerial uart_gps(RXPIN, TXPIN);

// This is where you declare prototypes for the functions that will be 
// using the TinyGPS library.
void getgps(TinyGPS &gps)
{
  gps.f_get_position(&lat, &lng);
}


// Compass setup //
int compassAddress = 0x42 >> 1; // From datasheet compass address is 0x42
// shift the address 1 bit right, the Wire library only needs the 7
// most significant bits for the address
int reading = 0; 


// Anemometer setup //
#define WindSensorPin (2) // The pin location of the anemometer sensor
#define WindVanePin (A3) // The pin the wind vane sensor is connected to
#define VaneOffset 0; // define the anemometer offset from magnetic north

int VaneValue; // raw analog value from wind vane
int Direction; // translated 0 - 360 direction
int CalDirection; // converted value with offset applied
int LastValue; // last direction value

volatile bool IsSampleRequired; // this is set true every 2.5s. Get wind speed
volatile unsigned int TimerCount; // used to determine 2.5sec timer count
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in isr

float WindSpeed; // speed km/h


// isr handler for timer interrupt
void isr_timer() {

TimerCount++;

if(TimerCount == 3)
{
IsSampleRequired = true;
TimerCount = 0;
}
}

// This is the function that the interrupt calls to increment the rotation count
void isr_rotation() {

if((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
Rotations++;
ContactBounceTime = millis();
}
}


// Get Wind Direction
void getWindDirection() {

VaneValue = analogRead(WindVanePin);
Direction = map(VaneValue, 0, 1023, 0, 359);
CalDirection = Direction + VaneOffset;

if(CalDirection > 360)
CalDirection = CalDirection - 360;

if(CalDirection < 0)
CalDirection = CalDirection + 360;

}


void setup()
{
  // This is the serial rate for your terminal program. It must be this 
  // fast because we need to print everything before a new sentence 
  // comes in. If you slow it down, the messages might not be valid and 
  // you will likely get checksum errors.
  Serial.begin(115200);
  //Sets baud rate of your GPS
  uart_gps.begin(GPSBAUD);
  
  
  // join i2c bus (address optional for master) 
  Wire.begin(); 


  LastValue = 0;

  IsSampleRequired = false;

  TimerCount = 0;
  Rotations = 0; // Set Rotations to 0 ready for calculations

  pinMode(WindSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);

  // Setup the timer interupt
  Timer1.initialize(500000);// Timer interrupt every 2.5 seconds
  Timer1.attachInterrupt(isr_timer);
      
}

void loop()
{
  // GPS //
  while(uart_gps.available())     // While there is data on the RX pin...
  {
      int c = uart_gps.read();    // load the data into a variable...
      if(gps.encode(c))      // if there is a new valid sentence...
      {
        getgps(gps);         // then grab the data.
      }
  }
  
  

  // Compass //
  // step 1: instruct sensor to read echoes 
  Wire.beginTransmission(compassAddress);  // transmit to device
  // the address specified in the datasheet is 66 (0x42) 
  // but i2c adressing uses the high 7 bits so it's 33 
  Wire.write('A');        // command sensor to measure angle  
  Wire.endTransmission(); // stop transmitting 

  // step 2: wait for readings to happen 
  delay(10); // datasheet suggests at least 6000 microseconds 

  // step 3: request reading from sensor 
  Wire.requestFrom(compassAddress, 2); // request 2 bytes from slave device #33 

  // step 4: receive reading from sensor 
  if (2 <= Wire.available()) // if two bytes were received 
  { 
    reading = Wire.read();  // receive high byte (overwrites previous reading) 
    reading = reading << 8; // shift high byte to be high 8 bits 
    reading += Wire.read(); // receive low byte as lower 8 bits 
    reading /= 10;
  } 


  // Anemometer //
  getWindDirection();

  LastValue = CalDirection;


  if(IsSampleRequired) {
  // convert to mp/h using the formula V=P(2.25/T)
  // V = P(2.25/2.5) = P * 0.9
  WindSpeed = Rotations * 0.9 * 1.6; // convert to km/h
  Rotations = 0; // Reset count for next sample

  IsSampleRequired = false;
  }
///*
  Serial.print(lat,9);
  Serial.print(",");
  Serial.print(lng,9);
  Serial.print("@");
  Serial.print(reading);
  Serial.print("@");
  Serial.print(WindSpeed);
  Serial.print(",");
  Serial.println(CalDirection);
  delay (100);
// */
}
