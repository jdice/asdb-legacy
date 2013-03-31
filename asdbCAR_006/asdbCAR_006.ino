#include <TinyGPS.h>
#include <PWMServo.h>
#include <aJSON.h>
#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"

double goalSpeed = 6.5; //miles per hour
double speedTolerance = 0.5; // +/- miles per hour
const boolean debugPrint = false;
const boolean jsonPrint = true;
boolean drivetrainEnabled = false;
boolean adjustMySpeed = true;
boolean gpsLock = false;
boolean slowMode = true;
boolean brakeMode = false;
const unsigned long brakeDuration = 1600;
TinyGPS gps;
PWMServo speedControl;
float currentLat, currentLon, goalLat, goalLon, currentHeading, goalHeading, initialLat, initialLon, distanceToGoal;
const int lockLED =  13; 
unsigned long timer100, timer500, brakeTimer;
void gpsdump(TinyGPS &gps);
boolean feedgps();
void printFloat(double f, int digits = 2);
unsigned long timestarted; //------------------------------------------------
const int escPin = 11;
int pointid = 0;
char waypointBehavior = 's'; // r means reverse at end of goalpoints, l means loop through goalpoints, s means stop at final goalpoint
boolean waypointDirection = true; // true means traverse waypoints in order, false reverses the order, this is managed by the car
double currentSpeed = 0;
int speedValue = 100;
int adjustedSpeedValue = speedValue;
double maxSpeed;
char* debugInfo;
aJsonStream serial_stream(&Serial3);

//steering
boolean steeringEnabled = true;
PWMServo steeringServo;
const int centerSteeringTuningValue = 83; // lower means more right, higher means more left
int steeringAngle = centerSteeringTuningValue;
float turnAngle;

//bumpers
boolean leftBumper, rightBumper;
boolean bumperStop = false;
const int leftBumperPin = 25;
const int rightBumperPin = 23;

//headlight
const int headlightPin = 53;
boolean headlightEnabled = false;

//rangefinder
const int rangefinderPin = 0;
double rangefinderValue = 0;
boolean rangefinderStop = false;

//xbee variables
boolean xbeeStop = true;
boolean stopOverrides = false;

//imu gyro
float imuEulerAngles[3];
FreeSixIMU imuSixDOF = FreeSixIMU();

//drive battery monitor
const int driveBatteryMonitorPin = 10;
const float driveBatteryVoltageRatio = 2.175;
float driveBatteryVoltage = 0;

void setup()
{
  Serial3.begin(57600);
  Serial2.begin(9600);
  Serial2.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  Serial2.println(PMTK_SET_NMEA_UPDATE_10HZ);
  Wire.begin();
  imuSixDOF.init();
  
  if(debugPrint){
    Serial3.print("asdb is running.");
    Serial3.println();
  }
  debugInfo = "asdb is running.";
  steeringServo.attach(SERVO_PIN_A); //pin 11
  speedControl.attach(SERVO_PIN_B); //pin 12
  pinMode(lockLED, OUTPUT);
  pinMode(leftBumperPin, INPUT);
  pinMode(rightBumperPin, INPUT);
  pinMode(escPin, OUTPUT);
}

aJsonObject *createMessage()
{
  aJsonObject *msg = aJson.createObject();
    aJsonObject *myStatus = aJson.createObject();
      aJson.addBoolToObject(myStatus, "stopOverrides", stopOverrides);
      aJson.addBoolToObject(myStatus, "steeringEnabled", steeringEnabled);
      aJson.addBoolToObject(myStatus, "xbeeStop", xbeeStop);
      aJson.addBoolToObject(myStatus, "drivetrainEnabled", drivetrainEnabled);
      aJson.addBoolToObject(myStatus, "headlightEnabled", headlightEnabled);
      aJson.addBoolToObject(myStatus, "adjustMySpeed", adjustMySpeed);
      aJson.addBoolToObject(myStatus, "gpsLock", gpsLock);
    aJson.addItemToObject(msg, "status", myStatus);
    aJsonObject *mySensors = aJson.createObject();
      aJson.addNumberToObject(mySensors, "rangefinder", rangefinderValue);
      aJson.addBoolToObject(mySensors, "leftBumper", leftBumper);
      aJson.addBoolToObject(mySensors, "rightBumper", rightBumper);
      aJson.addNumberToObject(mySensors, "driveBatteryVoltage", driveBatteryVoltage);
    aJson.addItemToObject(msg, "sensors", mySensors);
    aJsonObject *myLocation = aJson.createObject();
      aJsonObject *myLocationCurrent = aJson.createObject();
        aJson.addNumberToObject(myLocationCurrent, "lat", currentLat);
        aJson.addNumberToObject(myLocationCurrent, "lon", currentLon);
      aJson.addItemToObject(myLocation, "current", myLocationCurrent);
      aJsonObject *myLocationGoal = aJson.createObject();
        aJson.addNumberToObject(myLocationGoal, "lat", goalLat);
        aJson.addNumberToObject(myLocationGoal, "lon", goalLon);
        aJson.addNumberToObject(myLocationGoal, "distanceToGoal", distanceToGoal);
      aJson.addItemToObject(myLocation, "goal", myLocationGoal);
      aJsonObject *myLocationHeading = aJson.createObject();
        aJson.addNumberToObject(myLocationHeading, "current", currentHeading);
        aJson.addNumberToObject(myLocationHeading, "goal", goalHeading);
        aJson.addNumberToObject(myLocationHeading, "turnAngle", turnAngle);
        aJson.addNumberToObject(myLocationHeading, "steeringAngle", steeringAngle);
      aJson.addItemToObject(myLocation, "heading", myLocationHeading);
      aJsonObject *myLocationSpeed = aJson.createObject();
        aJson.addNumberToObject(myLocationSpeed, "current", currentSpeed);
        aJson.addNumberToObject(myLocationSpeed, "goal", goalSpeed);
        aJson.addNumberToObject(myLocationSpeed, "max", maxSpeed);
        aJson.addNumberToObject(myLocationSpeed, "adjustedSpeedValue", adjustedSpeedValue);
      aJson.addItemToObject(myLocation, "speed", myLocationSpeed);
    aJson.addItemToObject(msg, "location", myLocation);
    aJsonObject *myIMU = aJson.createObject();
      aJson.addNumberToObject(myIMU, "yaw", imuEulerAngles[0]);
      aJson.addNumberToObject(myIMU, "pitch", imuEulerAngles[1]);
      aJson.addNumberToObject(myIMU, "roll", imuEulerAngles[2]);
      //aJson.addNumberToObject(myIMU, "accX", imuAccX);
      //aJson.addNumberToObject(myIMU, "accY", imuAccY);
      //aJson.addNumberToObject(myIMU, "accZ", imuAccZ);
    aJson.addItemToObject(msg, "imu", myIMU);
    aJson.addStringToObject(msg, "debug", debugInfo);
  return msg;
}

void printFloat(double number, int digits){
  if(number < 0.0){ // Handle negative numbers
    Serial3.print('-');
    number = -number;
  }
  double rounding = 0.5;  // Round correctly so that print(1.999, 2) prints as "2.00"
  for (uint8_t i=0; i<digits; ++i){
    rounding /= 10.0;
  }
  number += rounding;
  unsigned long int_part = (unsigned long)number;  // Extract the integer part of the number and print it
  double remainder = number - (double)int_part;
  Serial3.print(int_part);
  if(digits > 0){  // Print the decimal point, but only if there are digits beyond
    Serial3.print(".");
  }
  while (digits-- > 0){ // Extract digits from the remainder one at a time
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial3.print(toPrint);
    remainder -= toPrint; 
  }
}

float getTurnAngle(float goalHeading, float currentHeading){
  float turnAngle;
  turnAngle = goalHeading - currentHeading;
  if(turnAngle > 180){
    turnAngle -= 360;
  }else if(turnAngle < -180){
    turnAngle += 360;
  }
  return turnAngle;
}



void gpsdump(TinyGPS &gps){
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars, speed, course;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;
  unsigned long heading;  

  feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  gps.f_get_position(&flat, &flon, &age);
  currentLat = flat;
  currentLon = flon;

// Jewell Track with cutout at stands towards 50 line of field
//float goal[] = {39.24695, -94.41040, 3.0, 39.24676, -94.41022, 3.0, 39.24670, -94.40990, 3.0, 39.24679, -94.40964, 3.0, 39.24696, -94.40945, 3.0, 39.24748, -94.40943, 3.0, 39.24791, -94.40943, 3.0, 39.24804, -94.40952, 3.0, 39.24813, -94.40968, 3.0, 39.24820, -94.40990, 3.0, 39.24809, -94.41022, 3.0, 39.24775, -94.41030, 3.0, 39.24743, -94.41020, 3.0};

float goal[] = {
39.24736, -94.41043, 3.0,
39.24704, -94.41045, 3.0,
39.24680, -94.41031, 3.0,
39.24669, -94.41001, 3.0,
39.24674, -94.40968, 3.0,
39.24697, -94.40943, 3.0,
39.24742, -94.40943, 3.0,
39.24786, -94.40943, 3.0,
39.24808, -94.40961, 3.0,
39.24819, -94.40981, 3.0,
39.24810, -94.41021, 3.0,
39.24793, -94.41038, 3.0,
39.24752, -94.41044, 3.0};

/* old Jewell Track replaced 03/29/2013
float goal[] = {
39.24697, -94.41041, 3.0,
39.24676, -94.41022, 3.0,
39.24670, -94.40990, 3.0,
39.24679, -94.40964, 3.0,
39.24696, -94.40945, 3.0,
39.24748, -94.40943, 3.0,
39.24791, -94.40943, 3.0,
39.24804, -94.40952, 3.0, 
39.24813, -94.40968, 3.0, 
39.24820, -94.40990, 3.0, 
39.24809, -94.41021, 3.0, 
39.24785, -94.41038, 3.0, 
39.24740, -94.41041, 3.0};
*/
// old Jewell Track
// float goal[] = {39.24695, -94.41040, 3.0, 39.24676, -94.41022, 3.0, 39.24670, -94.40990, 3.0, 39.24679, -94.40964, 3.0, 39.24696, -94.40945, 3.0, 39.24748, -94.40943, 3.0, 39.24791, -94.40943, 3.0, 39.24804, -94.40952, 3.0, 39.24813, -94.40968, 3.0, 39.24820, -94.40990, 3.0, 39.24809, -94.41022, 3.0, 39.24785, -94.41040, 3.0, 39.24740, -94.41040, 3.0};

// old Jewell Track
// float goal[] = {39.24695, -94.41040, 3.0,39.24676, -94.41022, 3.0,39.24670, -94.40990, 3.0,39.24678, -94.40962, 3.0,39.24695, -94.40945, 3.0,39.24785, -94.40941, 3.0,39.24809, -94.40968, 3.0,39.24820, -94.40990, 3.0,39.24809, -94.41022, 3.0,39.24785, -94.41040, 3.0};

// old Jewell Track
// float goal[] = {39.24695, -94.41038, 5.0, 39.24678, -94.41020, 5.0, 39.24671, -94.40990, 5.0, 39.24678, -94.40962, 5.0, 39.24695, -94.40944, 5.0, 39.24785, -94.40940, 5.0, 39.24809, -94.40955, 5.0, 39.24818, -94.40990, 5.0, 39.24809, -94.41022, 5.0, 39.24785, -94.41038, 5.0}; // lat1, lon1, accuracy1, lat2, lon2, accuracy2, etc.

// Two random points at Jewell field, diagonal across the field are 39.24727, -94.41008, 5.0, 39.24765, -94.40979

// Center of Jewell field is 39.24745, -94.40990

// South half of arc 39.24695, -94.41038, 5.0, 39.24678, -94.41020, 5.0, 39.24671, -94.40990, 5.0, 39.24678, -94.40962, 5.0, 39.24695, -94.40944, 5.0, 
  speed = gps.speed();
  currentSpeed = ((double) speed) * 0.011;
  heading = gps.course();
  currentHeading = heading/100;
  goalLat = goal[pointid];
  goalLon = goal[pointid+1];
  goalHeading = gps.course_to(currentLat, currentLon, goalLat, goalLon);
  turnAngle = getTurnAngle(goalHeading, currentHeading);
  if(initialLat==0){
    initialLat = flat;
  }
  if(initialLon==0){
    initialLon = flon;
  }
  
  if(turnAngle <= 0){
    steeringAngle = map(turnAngle, -180, 0, 116, centerSteeringTuningValue);
  }else{
    steeringAngle = map(turnAngle, 0, 180, centerSteeringTuningValue, 64);
  }
  
  steeringAngle = map(turnAngle, -180, 180, 116, 64); // we think 116 and 64 are the physical bounds on the steering capabilities. Change the "degree bounds" as desired
  distanceToGoal = gps.distance_between(currentLat, currentLon, goalLat, goalLon);
  if(distanceToGoal < goal[pointid+2]){ //check if location is within accuracy range of point
    if(waypointBehavior == 's'){
      if((pointid+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, start course from the first goalpoint
        pointid += 3;
      }else{
        goalSpeed = 0;
        drivetrainEnabled = false;
      }
    }else if(waypointBehavior == 'r'){
      if(waypointDirection){
        if((pointid+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, start course from the first goalpoint
          pointid += 3;
        }else{
          waypointDirection = false;
          if((pointid-3) > -1){
            pointid -=3;
          }
        }
      }else{
        if((pointid-3) > -1){
          pointid -=3;
        }else{
          waypointDirection = true;
          if((pointid+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, start course from the first goalpoint
            pointid += 3;
          }
        }
      }       
    }else{
      if((pointid+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, start course from the first goalpoint
        pointid += 3;
      }else{
        pointid = 0;
      }
    }
  }
  if(debugPrint){
    Serial3.print(" Current Goal:");
    printFloat(goal[pointid], 5);
    Serial3.print(", ");
    printFloat(goal[pointid+1], 5);
    Serial3.println();
    Serial3.print(" Distance to Current Goal:");
    Serial3.print(gps.distance_between(currentLat, currentLon, goal[pointid], goal[pointid+1]));
    Serial3.println();
    Serial3.println();
    Serial3.print("Lat/Long(float): ");
    printFloat(flat, 5);
    Serial3.print(", ");
    printFloat(flon, 5);
    Serial3.print(" Fix age: ");
    Serial3.print(age);
    Serial3.print(" ms.");
    Serial3.println();
    Serial3.print(" Heading:");
    Serial3.print(currentHeading);
    Serial3.print(" degrees.");
    Serial3.println();
    Serial3.print(" Course_to:");
    Serial3.print(goalHeading);
    Serial3.print(" degrees.");
    Serial3.println();
    Serial3.print(" Turn Angle:");
    Serial3.print(turnAngle);
    Serial3.print(" degrees.");
    Serial3.println();
    Serial3.print(" Steering Servo:");
    Serial3.print(steeringAngle);
    Serial3.println();
    Serial3.print("Speed: ");
    Serial3.print(currentSpeed); 
    Serial3.print("MPH");
    Serial3.println();
    Serial3.print("Speed Value: ");
    Serial3.print(speedValue);
    Serial3.println();
    Serial3.print("Maximum Speed Attained: ");
    Serial3.print(maxSpeed);
    Serial3.println();
    Serial3.println(rangefinderValue);
  }

  feedgps();

}



boolean feedgps(){
  while (Serial2.available()){
    if (gps.encode(Serial2.read())){
      return true;
    }
  }
  return false;
}

void adjustSpeed(){
  if(adjustMySpeed){
    if( (currentSpeed > (goalSpeed + speedTolerance)) && (speedValue < 85) ){
      speedValue++;
      if(debugPrint){
        Serial3.println("\nSLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER!\n");
      }
    }
    if( (currentSpeed < (goalSpeed - speedTolerance)) && (speedValue > 65) ){
      speedValue--;
      if(debugPrint){
        Serial3.println("\nFASTER -- FASTER -- FASTER -- FASTER -- FASTER -- FASTER -- FASTER -- FASTER -- FASTER -- FASTER -- FASTER!\n");
      }
    }
  }
}

void readSensors(){
  //Bumpers
  if(digitalRead(leftBumperPin) == HIGH){
    leftBumper = true;
  }else{
    leftBumper = false;
  }
  if(digitalRead(rightBumperPin) == HIGH){
    rightBumper = true;
  }else{
    rightBumper = false;
  }
  if(leftBumper || rightBumper){
    bumperStop = true;
  }else{
    bumperStop = false;
  }
  
  // Rangefinder
  rangefinderValue = analogRead(rangefinderPin)/1.8;
  if(rangefinderValue < 100){
    rangefinderStop = true;
  }else{
    rangefinderStop = false;
  }
  
  // IMU 6dof
  imuSixDOF.getEuler(imuEulerAngles);
  
  // Drive Battery Monitor
  driveBatteryVoltage = analogRead(driveBatteryMonitorPin) * 0.00418 * driveBatteryVoltageRatio;
}
void loop()
{
  ////////////////////////////////////////////////////////////////////////////////////////
  while(Serial3.available()){  //is there anything to read?
    char getData = Serial3.read();  //if yes, read it
    if(getData == 'q'){
      xbeeStop = true;
    }else if(getData == 'w'){
      xbeeStop = false;
    }else if(getData == 'h'){
      headlightEnabled = !headlightEnabled;
    }else if(getData == 'e'){
      slowMode = !slowMode;
    }else if(getData == 'r'){
      stopOverrides = !stopOverrides;
    }
  }
//////////////////////////////////////////////////////////////////////////////////////
  boolean newdata = false;
  unsigned long start = millis();
  readSensors();
  
  //stop criteria
  if(!stopOverrides){
    if(xbeeStop || rangefinderStop || bumperStop){ // list all stopping conditions here with a boolean OR between
      if(drivetrainEnabled){
        brakeMode = true;
        brakeTimer = millis()+brakeDuration;
      }
      drivetrainEnabled = false;
    }else{ // if NONE of the stopping conditions are met, enable drivetrain
      drivetrainEnabled = true;
    }
  }else{
    if(xbeeStop){ // list all stopping conditions here with a boolean OR between
      if(drivetrainEnabled){
        brakeMode = true;
        brakeTimer = millis()+brakeDuration;
      }
      drivetrainEnabled = false;
    }else{ // if NONE of the stopping conditions are met, enable drivetrain
      drivetrainEnabled = true;
    }
  }
  
  if(headlightEnabled){
    digitalWrite(headlightPin, HIGH);
  }else{
    digitalWrite(headlightPin, LOW);
  }
  
  if(drivetrainEnabled){
    adjustedSpeedValue = speedValue;
    if(adjustMySpeed == false){
      if(slowMode){
        speedValue = 89;
      }else{
        speedValue = 25;
      }
      adjustMySpeed = false;
    }
  }else{
    adjustMySpeed = false;
    if(millis() > brakeTimer){
      brakeMode = false;
    }
    if(brakeMode){
      adjustedSpeedValue = 110; // brake the car for brakeDuration
    }else{
      adjustedSpeedValue = 100; // stop the car if drivetrain is disabled.
    }
  }   
  
  speedControl.write(adjustedSpeedValue);
  
  while (millis() - start < 100)
  {
    if(currentSpeed > maxSpeed){
      maxSpeed = currentSpeed;
    }
    if (feedgps())
      newdata = true;
      
  }
  if(millis() - timer100 > 100){
    timer100 = millis();
    if(steeringEnabled){
      steeringServo.write(steeringAngle);
    }
  }
  
  if(millis() - timer500 > 500){
    adjustSpeed();
    if(jsonPrint){
      aJsonObject *msg = createMessage();
      aJson.print(msg, &serial_stream);
      Serial3.print("~");
      Serial3.println();
      aJson.deleteItem(msg);
    }
    timer500 = millis();
  }
  
  if (newdata){
    digitalWrite(lockLED, HIGH);
    gpsLock = true;
    if(debugPrint){
      Serial3.println();
      Serial3.println("-------------");
      Serial3.println("Acquired Data");
      Serial3.println("-------------");
    }
    gpsdump(gps);
  }else{
    digitalWrite(lockLED, LOW);
    gpsLock = false;
  }
}
