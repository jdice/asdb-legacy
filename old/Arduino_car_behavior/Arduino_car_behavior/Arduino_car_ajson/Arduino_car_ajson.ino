#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <PWMServo.h>
#include <aJSON.h>
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"


double targetSpeed = 6.5; //miles per hour
double speedTolerance = 0.5; // +/- miles per hour
boolean debugPrint = false;//true;
boolean steeringEnabled = true;
boolean drivetrainEnabled = false;
boolean headlightEnabled = true;
boolean adjustMySpeed = true;
TinyGPS gps;

PWMServo steeringServo;
PWMServo speedControl;
const int bumper = 7;
int bumperState = 0;
SoftwareSerial mySerial(2, 3);
float myLat, myLon;
float initialLat = 0;
float initialLon = 0;
float directionDifference;
int steeringAngle;
int lockLED =  13; 
unsigned long timer100, timer500;
void gpsdump(TinyGPS &gps);
boolean feedgps();
void printFloat(double f, int digits = 2);
unsigned long timestarted; //------------------------------------------------
int drive = 11; //pin 11
int pointid = 0;
char waypointBehavior = 's'; // r means reverse at end of goalpoints, l means loop through goalpoints, s means stop at final goalpoint
boolean goingForward = true;
boolean leftBumper, rightBumper;
int leftBumperPin = 6;
int rightBumperPin = 7;
int leftBumperLED = 8;
int rightBumperLED = 11;
double speedmph;
int speedValue = 87;
int adjustedSpeedValue = speedValue;
double maxSpeedAttained;

//xbee variables
boolean xbeeStop = false;

void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600);
  mySerial.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  mySerial.println(PMTK_SET_NMEA_UPDATE_10HZ);
  Serial.print("AutoSteerDuinoBot 3000 starting up...");
  Serial.println();
  steeringServo.attach(SERVO_PIN_A);
  speedControl.attach(SERVO_PIN_B);
  pinMode(lockLED, OUTPUT);
  pinMode(leftBumperPin, INPUT);
  pinMode(rightBumperPin, INPUT);
  pinMode(drive, OUTPUT);
  pinMode(leftBumperLED, OUTPUT);
  pinMode(rightBumperLED, OUTPUT);
  /*aJsonObject* message = aJson.createObject();
    aJsonObject* statusMessage = aJson.createObject();
      aJson.addTrueToObject(statusMessage, "steering");
      aJson.addTrueToObject(statusMessage, "drivetrain");
      aJson.addTrueToObject(statusMessage, "headlight");
      aJson.addTrueToObject(statusMessage, "adjustMySpeed");
      aJson.addFalseToObject(statusMessage, "gpsLock");
      aJson.addFalseToObject(statusMessage, "leftBumper");
      aJson.addFalseToObject(statusMessage, "rightBumper");
    aJson.addItemToObject(message, "status", statusMessage);
    aJsonObject* locationMessage = aJson.createObject();
      aJsonObject* currentLocation = aJson.createObject();
        aJson.addNumberToObject(currentLocation, "lat", (double) 0.0);
        aJson.addNumberToObject(currentLocation, "lon", (double) 0.0);
        aJson.addItemToObject(locationMessage, "current", currentLocation);
      aJsonObject* goalLocation = aJson.createObject();
        aJson.addNumberToObject(goalLocation, "lat", (double) 0.0);
        aJson.addNumberToObject(goalLocation, "lon", (double) 0.0);
        aJson.addItemToObject(locationMessage, "goal", goalLocation);
    aJson.addItemToObject(message, "location", locationMessage);
    aJsonObject* headingMessage = aJson.createObject();
      aJson.addNumberToObject(headingMessage, "current", (double) 0.0);
      aJson.addNumberToObject(headingMessage, "goal", (double) 0.0);
    aJson.addItemToObject(message, "heading", headingMessage);
    aJsonObject* speedMessage = aJson.createObject();
      aJson.addNumberToObject(speedMessage, "current", (double) 0.0);
      aJson.addNumberToObject(speedMessage, "target", (double) 0.0);
      aJson.addNumberToObject(speedMessage, "max", (double) 0.0);
    aJson.addItemToObject(message, "speed", speedMessage);
    aJsonObject* debugMessage = aJson.createObject();
      aJson.addNumberToObject(debugMessage, "speedController", 5);
      aJson.addNumberToObject(debugMessage, "steeringServo", 12);
    aJson.addItemToObject(message, "debug", debugMessage);
  char* theOutputtyThing = aJson.print(statusMessage);
  char* theOutputtyThing2 = aJson.print(debugMessage);
  //Serial.println(theOutputtyThing);
  //Serial.println(theOutputtyThing2);*/
  Serial.println("JSON outputted...?");
}



void printFloat(double number, int digits)
{
  if (number < 0.0) // Handle negative numbers
  {
     Serial.print('-');
     number = -number;
  }
  double rounding = 0.5;  // Round correctly so that print(1.999, 2) prints as "2.00"
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  number += rounding;
  unsigned long int_part = (unsigned long)number;  // Extract the integer part of the number and print it
  double remainder = number - (double)int_part;
  Serial.print(int_part);
  if (digits > 0)  // Print the decimal point, but only if there are digits beyond
    Serial.print("."); 
  while (digits-- > 0)  // Extract digits from the remainder one at a time
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint; 
  } 
}



float getTurnAngle(float courseHeading, float currentHeading){
  float directionDifference;
  directionDifference = courseHeading - currentHeading;
  if (directionDifference  > 180){
    directionDifference -= 360;
  }else if (directionDifference  < -180){
    directionDifference += 360;
  }
  return directionDifference;
}



void gpsdump(TinyGPS &gps){
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars, speed, course;
  float courseHeading;
  float currentHeading;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;
  unsigned long heading;  

  feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  gps.f_get_position(&flat, &flon, &age);
  myLat = flat;
  myLon = flon;

// Jewell Track with cutout at stands towards 50 line of field
//float goal[] = {39.24695, -94.41040, 3.0, 39.24676, -94.41022, 3.0, 39.24670, -94.40990, 3.0, 39.24679, -94.40964, 3.0, 39.24696, -94.40945, 3.0, 39.24748, -94.40943, 3.0, 39.24791, -94.40943, 3.0, 39.24804, -94.40952, 3.0, 39.24813, -94.40968, 3.0, 39.24820, -94.40990, 3.0, 39.24809, -94.41022, 3.0, 39.24775, -94.41030, 3.0, 39.24743, -94.41020, 3.0};

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
  speedmph = ((double) speed) * 0.011;
  heading = gps.course();
  currentHeading = heading/100;
  courseHeading = gps.course_to(myLat, myLon, goal[pointid], goal[pointid+1]);
  directionDifference = getTurnAngle(courseHeading, currentHeading);
  if(initialLat==0){
    initialLat = flat;
  }
  if(initialLon==0){
    initialLon = flon;
  }
  steeringAngle = map(directionDifference, -190, 180, 116, 64); //right map was 64, tuned it straighter
  if(gps.distance_between(myLat, myLon, goal[pointid], goal[pointid+1]) < goal[pointid+2]){ //check if location is within accuracy range of point
    if(waypointBehavior == 's'){
      if((pointid+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, start course from the first goalpoint
        pointid += 3;
      }else{
        targetSpeed = 0;
        drivetrainEnabled = false;
      }
    }else if(waypointBehavior == 'r'){
      if(goingForward){
        if((pointid+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, start course from the first goalpoint
          pointid += 3;
        }else{
          goingForward = false;
          if((pointid-3) > -1){
            pointid -=3;
          }
        }
      }else{
        if((pointid-3) > -1){
          pointid -=3;
        }else{
          goingForward = true;
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
  /*if(debugPrint){ //pre-json
    Serial.print(" Current Goal:");
    printFloat(goal[pointid], 5);
    Serial.print(", ");
    printFloat(goal[pointid+1], 5);
    Serial.println();
    Serial.print(" Distance to Current Goal:");
    Serial.print(gps.distance_between(myLat, myLon, goal[pointid], goal[pointid+1]));
    Serial.println();
    Serial.println();
    Serial.print("Lat/Long(float): ");
    printFloat(flat, 5);
    Serial.print(", ");
    printFloat(flon, 5);
    Serial.print(" Fix age: ");
    Serial.print(age);
    Serial.print(" ms.");
    Serial.println();
    Serial.print(" Heading:");
    Serial.print(currentHeading);
    Serial.print(" degrees.");
    Serial.println();
    Serial.print(" Course_to:");
    Serial.print(courseHeading);
    Serial.print(" degrees.");
    Serial.println();
    Serial.print(" Directional Difference:");
    Serial.print(directionDifference);
    Serial.print(" degrees.");
    Serial.println();
    Serial.print(" Steering Servo:");
    Serial.print(steeringAngle);
    Serial.println();
    Serial.print("Speed: ");
    Serial.print(speedmph); 
    Serial.print("MPH");
    Serial.println();
    Serial.print("Speed Value: ");
    Serial.print(speedValue);
    Serial.println();
    Serial.print("Maximum Speed Attained: ");
    Serial.print(maxSpeedAttained);
    Serial.println();
  }*/   

  feedgps();

}



boolean feedgps(){
  while (mySerial.available()){
    if (gps.encode(mySerial.read())){
      return true;
    }
  }
  return false;
}

void adjustSpeed(){
  if(adjustMySpeed){
    if( (speedmph > (targetSpeed + speedTolerance)) && (speedValue < 98) ){
      speedValue++;
      //Serial.println("\nSLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER -- SLOWER!\n");
    }
    if( (speedmph < (targetSpeed - speedTolerance)) && (speedValue > 65) ){
      speedValue--;
      //Serial.println("\nFASTER -- FASTER -- FASTER -- FASTER -- FASTER -- FASTER -- FASTER -- FASTER -- FASTER -- FASTER -- FASTER!\n");
    }
  }
}

void readBumpers(){
    if(digitalRead(leftBumperPin) == HIGH){
        leftBumper = true;
        digitalWrite(leftBumperLED,HIGH);
    }else{
        leftBumper = false;
        digitalWrite(leftBumperLED,LOW);
    }
    if(digitalRead(rightBumperPin) == HIGH){
        rightBumper = true;
        digitalWrite(rightBumperLED,HIGH);
    }else{
        rightBumper = false;
        digitalWrite(rightBumperLED,LOW);
    }
}
void loop()
{
  ////////////////////////////////////////////////////////////////////////////////////////
  while(Serial.available()){  //is there anything to read?
    char getData = Serial.read();  //if yes, read it
    if(getData == 'q'){
      xbeeStop = true;
    }else if(getData == 'w'){
      xbeeStop = false;
    }else if(getData == 'h'){
      headlightEnabled = !headlightEnabled;
    }
  }
//////////////////////////////////////////////////////////////////////////////////////
  boolean newdata = false;
  unsigned long start = millis();
  readBumpers();
  
  if(leftBumper || rightBumper || xbeeStop){ // list all stopping conditions here
    drivetrainEnabled = false;
  }else{ // if NONE of the stopping conditions are met, enable drivetrain
    drivetrainEnabled = true;
  }
  
  if(headlightEnabled){
    digitalWrite(5, HIGH);
  }else{
    digitalWrite(5, LOW);
  }
  
  if(drivetrainEnabled){
    adjustedSpeedValue = speedValue;
    if(adjustMySpeed == false){
      speedValue = 87;
      adjustMySpeed = true;
    }
  }else{
    adjustMySpeed = false;
    adjustedSpeedValue = 100; // stop the car if drivetrain is disabled.
  }   
  
  speedControl.write(adjustedSpeedValue);
  
  while (millis() - start < 100)
  {
    if(speedmph > maxSpeedAttained){
      maxSpeedAttained = speedmph;
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
    timer500 = millis();
  }
  
  if (newdata){
    digitalWrite(lockLED, HIGH); //LED ON
    if(debugPrint){
      Serial.println();
      Serial.println("-------------");
      Serial.println("Acquired Data");
      Serial.println("-------------");
      gpsdump(gps);
      
    }
  }else{
    digitalWrite(lockLED, LOW); //LED OFF
  }
}
