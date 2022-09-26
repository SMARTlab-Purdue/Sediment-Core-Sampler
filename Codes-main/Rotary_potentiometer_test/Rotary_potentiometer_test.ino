/*
 * Bas on Tech
 * This course is part of the courses on https://arduino-tutorials.net
 *  
 * (c) Copyright 2018-2019 - Bas van Dijk / Bas on Tech
 * This code and course is copyrighted. It is not allowed to use these courses commerically
 * without explicit written approval
 * 
 * YouTube:    https://www.youtube.com/c/BasOnTech
 * Facebook:   https://www.facebook.com/BasOnTechChannel
 * Instagram:  https://www.instagram.com/BasOnTech
 * Twitter:    https://twitter.com/BasOnTech
 * 
 */

int16_t potPin = A2;              // Potmeter pin
//int ledPin = LED_BUILTIN;     // Builtin LED pin
int16_t potVal = 0;               // Potmeter's value (0 by default)

void setup() {
  Serial.begin(9600);         // Start the serial monitor at 9600 baud
  //pinMode(ledPin, OUTPUT);    // Set ledPin as output
  pinMode(potPin, INPUT);
  delay(100);
}

// The loop() function runs infinitely
void loop() {
  
  potVal = analogRead(potPin);       // Read the analog value of the potmeter (0-1023)
  float ang = (float)potVal * 270.0 / 1023.0;
  Serial.println(potVal);            // Write the value to the serial monitor
  Serial.print("  ");
  Serial.println(ang);
  //digitalWrite(LED_BUILTIN, HIGH);   // Turn the built-in LED on
  //delay(potVal);                     // Pause for the length of the potval value (0-1023) milliseconds
  
  //digitalWrite(LED_BUILTIN, LOW);    // Turn the built-in LED on
  //delay(potVal);                     // Pause for the length of the potval value (0-1023) milliseconds
  delay(100);
}
