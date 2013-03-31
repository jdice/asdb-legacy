const int button1pin = 1;
const int button2pin = 2;
const int button3pin = 3;

int button1 = 0;
int button2 = 0;
int button3 = 0;


const int IRpin = 0;
const int transmitLEDpin = 1;

int PWMvalue;


void setup() {
  pinMode(button1pin, INPUT); 
  pinMode(button2pin, INPUT); 
  pinMode(button3pin, INPUT); 
  pinMode(IRpin, OUTPUT); 
  pinMode(transmitLEDpin, OUTPUT); 
}

void loop() {
  button1 = digitalRead(button1pin);
  button2 = digitalRead(button2pin);
  button3 = digitalRead(button3pin);
  
  if(button1 == true){
    PWMvalue = 25;
  }else if(button2 == true){
    PWMvalue = 100;
  }else if(button3 == true){
    PWMvalue = 175;
  }else{
    PWMvalue = 0;
  }

  if(PWMvalue != 0){
    digitalWrite(transmitLEDpin, HIGH);
  }else{
    digitalWrite(transmitLEDpin, LOW);
  }
  
  analogWrite(IRpin, PWMvalue);
}
