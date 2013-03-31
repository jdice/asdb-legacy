/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */

#include <Servo.h>
Servo myservo;
void setup() {                
  // initialize the digital pin as an output.
  myservo.attach(9);
  pinMode(6, OUTPUT);     
}

void loop() {
  myservo.write(89);
  delay(5000); 
  analogWrite(6, 1);  // set the LED on
  delay(1000);  // wait for a second
  myservo.write(122);
  delay(5000);
  digitalWrite(6, LOW);
  delay(200000);
  
}
