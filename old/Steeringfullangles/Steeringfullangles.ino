// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
 
   // variable to read the value from the analog pin 
 
void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
} 
 
void loop() 
{ 
            // reads the value of the potentiometer (value between 0 and 1023) 
      // scale it to use it with the servo (value between 0 and 180) 
  
  myservo.write(116);
  delay(1000);
  myservo.write(90);  // sets the servo position according to the scaled value 
  delay(1000); 
  myservo.write(64);
  delay(1000);
  myservo.write(90);  // sets the servo position according to the scaled value 
  delay(1000);
} 
