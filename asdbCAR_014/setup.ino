void setup(){
//  Serial.begin(9600);
  Serial.begin(57600);
  Serial2.begin(115200); // GPS
//  Serial3.begin(57600); // XBee
  Wire.begin();
  imuSixDOF.init();
  compass = HMC5883L();
  compass.SetScale(1.3);
  compass.SetMeasurementMode(Measurement_Continuous);
  debugInfo = "asdb is running.";
  steeringServo.attach(SERVO_PIN_A); //pin 11
  speedControl.attach(SERVO_PIN_B); //pin 12
  pinMode(lockLED, OUTPUT);
  pinMode(leftBumperPin, INPUT);
  pinMode(rightBumperPin, INPUT);
  pinMode(startButtonPin, INPUT_PULLUP);
}
