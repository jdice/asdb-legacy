#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Servo2.h>
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"


/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of NewSoftSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 2(rx) and 3(tx).
*/
boolean debugPrint = true;
boolean steeringEnabled = true;
TinyGPS gps;
Servo steeringServo;
SoftwareSerial mySerial(2, 3);
float myLat, myLon;
float initialLat = 0;
float initialLon = 0;
float directionDifference ;
int steeringAngle;
unsigned long currentTime;
void gpsdump(TinyGPS &gps);
boolean feedgps();
void printFloat(double f, int digits = 2);



void setup()
{
  Serial.begin(115200);
  mySerial.begin(9600);
  mySerial.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  mySerial.println(PMTK_SET_NMEA_UPDATE_1HZ);
  Serial.print("Welcome to the AutoSteerDuinoBot 3000. Starting up...");
  Serial.println();
  currentTime = millis();
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



void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars, course;
  float courseHeading;
  float currentHeading;
  //unsigned long age, time, date, speed, course;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;
  unsigned long heading;  
  if(steeringServo.attached())
  {
    steeringServo.detach();
  }  
  feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors
  if(steeringServo.attached())
  {
    steeringServo.detach();
  }
  gps.f_get_position(&flat, &flon, &age);
  myLat = flat;
  myLon = flon;
  float goal[] = {39.26450, -94.39680}; //end of the driveway at the gate
  heading = gps.course();
  currentHeading = heading/100;
  courseHeading = gps.course_to(myLat, myLon, goal[0], goal[1]);
  directionDifference = getTurnAngle(courseHeading, currentHeading);
  if(initialLat==0){
    initialLat = flat;
  }
  if(initialLon==0){
    initialLon = flon;
  }
  steeringAngle = map(directionDifference, -180, 180, 122, 70);
  if(debugPrint){
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
  }  
  if(steeringServo.attached())
  {
    steeringServo.detach();
  }
  feedgps();
  /*gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour)); Serial.print(":"); Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second)); Serial.print("."); Serial.print(static_cast<int>(hundredths));
  Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");  
  feedgps();
  */
}



boolean feedgps()
{
  if(steeringServo.attached())
  {
    steeringServo.detach();
  }
  while (mySerial.available())
  {
    if (gps.encode(mySerial.read()))
      return true;
  }
  return false;
}



void loop()
{
  boolean newdata = false;
  unsigned long start = millis();

  // Every 5 seconds we print an update
  while (millis() - start < 100)
  {
    if(steeringServo.attached())
      {
        steeringServo.detach();
      }
    if (feedgps())
      newdata = true;
  }
  if(millis() - currentTime > 751){
    currentTime = millis();
    if(steeringEnabled){
      steeringServo.attach(9);
      delay(29);
      steeringServo.write(steeringAngle);
      delay(101);
    }
  }
  if (newdata)
  {
    if(debugPrint){
      Serial.println("Acquired Data");
      Serial.println("-------------");
    }
    gpsdump(gps);
    Serial.println("-------------");
    //Serial.println((initialLat-myLat)*10000);
  }
}

