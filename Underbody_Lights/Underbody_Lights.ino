int val;

void setup() {
  Serial.begin(9600);    	//initialize serial
  pinMode(19, OUTPUT);   	//set pin 13 as output
}

void loop() {
  while(Serial.available()){  //is there anything to read?
    char getData = Serial.read();  //if yes, read it
    if(getData == 'a'){
      val+=10;
    }else if(getData == 'b'){
      val-=10;
    }
  }
  val = 128 + 127*sin( ( (float) millis() )/500);
  if(val > 255){
    val = 255;
  }
  if(val < 0 ){
    val = 0;
  }
  analogWrite(9,val);
  Serial.println(val);
}
