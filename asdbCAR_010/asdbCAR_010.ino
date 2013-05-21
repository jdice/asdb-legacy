#include <TinyGPS.h>
#include <PWMServo.h>
#include <aJSON.h>
#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>  
#include <FIMU_ITG3200.h>

int loopNumber = 0;
double goalSpeed = 6.5; //miles per hour
double speedTolerance = 0.5; // +/- miles per hour
const boolean jsonPrint = true;
boolean drivetrainEnabled = false;
boolean adjustMySpeed = true;
boolean gpsLock = false;
boolean slowMode = true;
boolean brakeMode = false;
const unsigned long brakeDuration = 1600;
TinyGPS gps;
PWMServo speedControl;
float currentLat, currentLon, goalLat, goalLon;
float currentHeading, currentGPSHeading, lastKnownGoodGPSHeading, imuYawStandard, imuHeading, goalHeading, adjustedGoalHeading;
float initialLat, initialLon, distanceToGoal;
const int imuHistorySize = 160;
float imuHistory[imuHistorySize];
float imuHistoryAverage, imuHistoryStdev;
const int gpsHeadingHistorySize = 50;
float gpsHeadingHistory[gpsHeadingHistorySize];
float gpsHeadingHistoryAverage, gpsHeadingHistoryStdev;
const int lockLED =  13;
unsigned long timer100, timer500, brakeTimer;
int pointid = 0;
char waypointBehavior = 'l'; // r means reverse at end of goalpoints, l means loop through goalpoints, s means stop at final goalpoint
boolean waypointDirection = true; // true means traverse waypoints in order, false reverses the order, this is managed by the car
double currentSpeed = 0;
int speedValue = 100;
int adjustedSpeedValue = speedValue;
double maxSpeed;
char* debugInfo;
aJsonStream serial_stream(&Serial3);

//goal points
// Jewell Track Center Line calibrated with new GPS at 2013-05-19 at 3:41pm
/*float goal[] = {
39.24736, -94.41039, 3.0,
39.24730, -94.41040, 3.0,
39.24722, -94.41041, 3.0,
39.24712, -94.41040, 3.0,
39.24698, -94.41037, 3.0,
39.24685, -94.41030, 3.0,
39.24673, -94.41006, 3.0,
39.24671, -94.40988, 3.0,
39.24673, -94.40971, 3.0,
39.24681, -94.40960, 3.0,
39.24691, -94.40950, 3.0,
39.24708, -94.40944, 3.0,
39.24719, -94.40944, 3.0,
39.24733, -94.40943, 3.0,
39.24750, -94.40943, 3.0,
39.24777, -94.40942, 3.0,
39.24791, -94.40944, 3.0,
39.24807, -94.40955, 3.0,
39.24817, -94.40973, 3.0,
39.24818, -94.40980, 3.0,
39.24816, -94.41002, 3.0,
39.24804, -94.41029, 3.0,
39.24780, -94.41040, 3.0,
39.24755, -94.41040, 3.0};*/


//Jewell center of field with right angles
float goal[] = {
39.24737, -94.41014, 3.0,
39.24720, -94.41017, 3.0,
39.24720, -94.40987, 3.0,
39.24736, -94.40986, 3.0};


//steering
boolean steeringEnabled = true;
PWMServo steeringServo;
const int centerSteeringTuningValue = 90; // lower means more right, higher means more left
const int fullRightSteeringTuningValue = 58;
const int fullLeftSteeringTuningValue = 116;
int steeringAngle = centerSteeringTuningValue;
float turnAngle = 0;
const float derivativeTuning = 0.9;

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
float imuValues[6];
const float imuAngularTrust = 5; // in degrees per second
FreeSixIMU imuSixDOF = FreeSixIMU();

//drive battery monitor
const int driveBatteryMonitorPin = 10;
const float driveBatteryVoltageRatio = 2.175;
float driveBatteryVoltage = 0;

void setup()
{
  Serial2.begin(115200);
  Serial3.begin(57600);
  Wire.begin();
  imuSixDOF.init();
  debugInfo = "asdb is running.";
  steeringServo.attach(SERVO_PIN_A); //pin 11
  speedControl.attach(SERVO_PIN_B); //pin 12
  pinMode(lockLED, OUTPUT);
  pinMode(leftBumperPin, INPUT);
  pinMode(rightBumperPin, INPUT);
  for(int i = 0; i<imuHistorySize; i++){
    imuHistory[i] = 0.0f;
  }
  for(int i = 0; i<gpsHeadingHistorySize; i++){
    gpsHeadingHistory[i] = 0.0f;
  }
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
      aJson.addNumberToObject(myStatus, "loopNumber", loopNumber);
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
        aJson.addNumberToObject(myLocationHeading, "lastGPS", lastKnownGoodGPSHeading);
        aJson.addNumberToObject(myLocationHeading, "gpsHeadingHistoryStdev", gpsHeadingHistoryStdev);
        aJson.addNumberToObject(myLocationHeading, "goal", goalHeading);
        aJson.addNumberToObject(myLocationHeading, "adjustedGoal", adjustedGoalHeading);
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
      aJson.addNumberToObject(myIMU, "yawstandard", imuYawStandard);
      aJson.addNumberToObject(myIMU, "imuHeading", imuHeading);
      aJson.addNumberToObject(myIMU, "imuHistoryAverage", imuHistoryAverage);
      aJson.addNumberToObject(myIMU, "imuHistoryStdev", imuHistoryStdev);
      aJson.addNumberToObject(myIMU, "pitch", imuEulerAngles[1]);
      aJson.addNumberToObject(myIMU, "roll", imuEulerAngles[2]);
      //aJson.addNumberToObject(myIMU, "accX", imuAccX);
      //aJson.addNumberToObject(myIMU, "accY", imuAccY);
      //aJson.addNumberToObject(myIMU, "accZ", imuAccZ);
    aJson.addItemToObject(msg, "imu", myIMU);
    aJson.addStringToObject(msg, "debug", debugInfo);
  return msg;
}

float getTurnAngle(float goalHeading, float currentHeading){
  float turnAngle;
  turnAngle = goalHeading - currentHeading;
  turnAngle = getCoterminalAngle(turnAngle);
  return turnAngle;
}

float adjustGoalHeading(){
  adjustedGoalHeading += derivativeTuning*getTurnAngle(goalHeading,adjustedGoalHeading);
  while(adjustedGoalHeading >= 360){
    adjustedGoalHeading -= 360;
  }
  while(adjustedGoalHeading < 0){
    adjustedGoalHeading += 360;
  }
}

float getCoterminalAngle(float angle){
  float coterminalAngle = angle;
  while(coterminalAngle > 180){
    coterminalAngle -= 360;
  }
  while(coterminalAngle < -180){
    coterminalAngle += 360;
  }
  return coterminalAngle;
}


void gpsdump(TinyGPS &gps){
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars, speed, course;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;
  unsigned long heading;  
  
  if(initialLat==0){
    initialLat = flat;
  }
  if(initialLon==0){
    initialLon = flon;
  }
  
  gps.f_get_position(&flat, &flon, &age);
  currentLat = flat;
  currentLon = flon;
  speed = gps.speed();
  currentSpeed = ((double) speed) * 0.011;
  heading = gps.course();
  currentGPSHeading = heading/100;
  
  for(int i=0; i<(gpsHeadingHistorySize-1); i++){
    gpsHeadingHistory[i+1] = gpsHeadingHistory[i];
  }
  gpsHeadingHistory[0] = currentGPSHeading;
  gpsHeadingHistoryAverage = 0;
  for(int i=0; i<gpsHeadingHistorySize; i++){
    gpsHeadingHistoryAverage += gpsHeadingHistory[i];
  }
  gpsHeadingHistoryAverage /= gpsHeadingHistorySize;
  gpsHeadingHistoryStdev = 0;
  for(int i=0; i<gpsHeadingHistorySize; i++){
    gpsHeadingHistoryStdev += sq(getCoterminalAngle(gpsHeadingHistory[i]-gpsHeadingHistoryAverage));
  }
  gpsHeadingHistoryStdev = sqrt(gpsHeadingHistoryStdev/(gpsHeadingHistorySize-1));
  
  if((currentSpeed > 1) && (imuHistoryStdev > 0) && (imuHistoryStdev < 2) && (abs(imuValues[3])<imuAngularTrust)){
    lastKnownGoodGPSHeading = currentGPSHeading;
    imuYawStandard = imuEulerAngles[0];
    imuHeading = imuEulerAngles[0]-imuYawStandard;
    currentHeading = currentGPSHeading;
  }else{
    imuHeading = imuEulerAngles[0]-imuYawStandard;
    //currentHeading = lastKnownGoodGPSHeading + imuHeading;
    currentHeading = currentGPSHeading;
  }
  
  while(currentHeading > 360){
    currentHeading -= 360;
  }
  while(currentHeading < 0){
    currentHeading += 360;
  }
  
  goalLat = goal[pointid];
  goalLon = goal[pointid+1];
  goalHeading = gps.course_to(currentLat, currentLon, goalLat, goalLon);
  adjustGoalHeading();
  turnAngle = getTurnAngle(adjustedGoalHeading, currentHeading);
  if(turnAngle <= 0){
    steeringAngle = map(turnAngle, -180, 0, fullLeftSteeringTuningValue, centerSteeringTuningValue);
  }else{
    steeringAngle = map(turnAngle, 0, 180, centerSteeringTuningValue, fullRightSteeringTuningValue);
  }
  
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
        if((pointid+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, reverse course
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
    }
    if( (currentSpeed < (goalSpeed - speedTolerance)) && (speedValue > 65) ){
      speedValue--;
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
  imuSixDOF.getValues(imuValues);
  if(loopNumber%5==0){
    for(int i=0; i<(imuHistorySize-1); i++){
      imuHistory[i+1] = imuHistory[i];
    }
    imuHistory[0] = imuValues[3];
    imuHistoryAverage = 0;
    for(int i=0; i<imuHistorySize; i++){
      imuHistoryAverage += imuHistory[i];
    }
    imuHistoryAverage /= imuHistorySize;
    imuHistoryStdev = 0;
    for(int i=0; i<imuHistorySize; i++){
      imuHistoryStdev += sq(getCoterminalAngle(imuHistory[i]-imuHistoryAverage));
    }
    imuHistoryStdev = sqrt(imuHistoryStdev/(imuHistorySize-1));
  }
  
  // Drive Battery Monitor
  driveBatteryVoltage = analogRead(driveBatteryMonitorPin) * 0.00418 * driveBatteryVoltageRatio;
}

void loop()
{
  loopNumber++;
  // Read from Xbee
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

  // Update sensor readings
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
  if(steeringEnabled){
      steeringServo.write(steeringAngle);
  }
  
  if (feedgps()){
    digitalWrite(lockLED, HIGH);
    gpsLock = true;
    gpsdump(gps);
  }else{
    digitalWrite(lockLED, LOW);
    gpsLock = false;
  }
  
  if(millis() - timer100 > 100){
    // Update max speed
    if(currentSpeed > maxSpeed){
      maxSpeed = currentSpeed;
    }
    
    // Update headlight
    if(headlightEnabled){
      digitalWrite(headlightPin, HIGH);
    }else{
      digitalWrite(headlightPin, LOW);
    }
    
    // Send JSON data
    if(jsonPrint){
      aJsonObject *msg = createMessage();
      aJson.print(msg, &serial_stream);
      Serial3.print("~");
      Serial3.println();
      aJson.deleteItem(msg);
    }
    loopNumber = 0;
    timer100 = millis();
  }
  
  if(millis() - timer500 > 500){
    adjustSpeed();
    timer500 = millis();
  }
}
