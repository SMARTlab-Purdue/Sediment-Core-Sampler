#include "TimerOne.h" // Timer Interrupt set to 2 second for read sensors
#include <math.h>

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

float WindSpeed; // speed miles per hour

void setup() {

LastValue = 0;

IsSampleRequired = false;

TimerCount = 0;
Rotations = 0; // Set Rotations to 0 ready for calculations

Serial.begin(9600);

pinMode(WindSensorPin, INPUT);
attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);

// Setup the timer interupt
Timer1.initialize(500000);// Timer interrupt every 2.5 seconds
Timer1.attachInterrupt(isr_timer);
}

void loop() {

getWindDirection();

LastValue = CalDirection;


if(IsSampleRequired) {
// convert to mp/h using the formula V=P(2.25/T)
// V = P(2.25/2.5) = P * 0.9
WindSpeed = Rotations * 0.9;
Rotations = 0; // Reset count for next sample

IsSampleRequired = false;

Serial.print(WindSpeed); Serial.print("\t\t");
Serial.print(CalDirection); Serial.println("\t");
}
}

// isr handler for timer interrupt
void isr_timer() {

TimerCount++;

if(TimerCount == 6)
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
