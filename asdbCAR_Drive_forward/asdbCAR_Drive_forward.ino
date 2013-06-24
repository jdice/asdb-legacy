#include <PWMServo.h>
PWMServo esc;  // create servo object to control a servo 
PWMServo steering;
int steeringValue = 76;
int slowSpeedValue = 89;
int reverseSpeedValue = 110;

void setup() {
  Serial.begin(57600);    	//initialize serial
  steering.attach(SERVO_PIN_A);
  esc.attach(SERVO_PIN_B); 	//set pin 13 as output
}

void loop() {
  while(Serial.available()){  //is there anything to read?
    char getData = Serial.read();  //if yes, read it
    if(getData == 'w'){
      esc.write(25);
      Serial.println("Fast!");
    }else if(getData == 's'){
      esc.write(100);
      Serial.println("Stop!");
    }else if(getData == 'e'){
      esc.write(slowSpeedValue);
      Serial.println("Slow!");
    }else if(getData =='r'){
      esc.write(reverseSpeedValue);
      Serial.println("Reverse reverse!");
    }else if(getData == 'o'){
      steeringValue++;
      Serial.println("Steer:");
      Serial.println(steeringValue);
    }else if(getData == 'p'){
      steeringValue--;
      Serial.println("Steer:");
      Serial.println(steeringValue);
    }else if(getData == 'k'){
      slowSpeedValue++;
      Serial.println("Slow Speed:");
      Serial.println(slowSpeedValue);
    }else if(getData == 'l'){
      slowSpeedValue--;
      Serial.println("Slow Speed:");
      Serial.println(slowSpeedValue);
    }
    steering.write(steeringValue);
  }
}
