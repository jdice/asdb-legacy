#include <PWMServo.h>
PWMServo esc;  // create servo object to control a servo 
PWMServo steering;
int steeringValue = 90;
int slowSpeedValue = 80;

void setup() {
  Serial3.begin(57600);    	//initialize serial
  steering.attach(SERVO_PIN_A);
  esc.attach(SERVO_PIN_B); 	//set pin 13 as output
}

void loop() {
  while(Serial3.available()){  //is there anything to read?
    char getData = Serial3.read();  //if yes, read it
    if(getData == 'w'){
      esc.write(25);
      Serial3.println("Fast!");
    }else if(getData == 's'){
      esc.write(100);
      Serial3.println("Stop!");
    }else if(getData == 'e'){
      esc.write(slowSpeedValue);
      Serial3.println("Slow!");
    }else if(getData == 'o'){
      steeringValue++;
      Serial3.println("Steer:");
      Serial3.println(steeringValue);
    }else if(getData == 'p'){
      steeringValue--;
      Serial3.println("Steer:");
      Serial3.println(steeringValue);
    }else if(getData == 'k'){
      slowSpeedValue++;
      Serial3.println("Slow Speed:");
      Serial3.println(slowSpeedValue);
    }else if(getData == 'l'){
      slowSpeedValue--;
      Serial3.println("Slow Speed:");
      Serial3.println(slowSpeedValue);
    }
    steering.write(steeringValue);
  }
}
