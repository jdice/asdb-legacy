#include <SoftwareSerial.h>
#include <TinyGPS.h>
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"


/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of NewSoftSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 2(rx) and 3(tx).
*/

TinyGPS gps;
SoftwareSerial mySerial(2, 3);
float myLat, myLon;
float initialLat = 0;
float initialLon = 0;
float directionDifference ;
int steeringServo ;
void gpsdump(TinyGPS &gps);
boolean feedgps();
void printFloat(double f, int digits = 2);

void setup()
{
  Serial.begin(115200);
  mySerial.begin(9600);
  mySerial.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  mySerial.println(PMTK_SET_NMEA_UPDATE_10HZ);
  Serial.print("Yo, starting up.");    

}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0)
  {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
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
  
  feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

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
  steeringServo = map(directionDifference, -180, 180, 70, 122);
  
  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
  Serial.print(" Fix age: ");
  Serial.print(age);
  Serial.print(" ms.");
  Serial.println(" Heading:");
  Serial.print(currentHeading);
  Serial.print(" degrees.");
  Serial.println(" Course_to:");
  Serial.print(courseHeading);
  Serial.print(" degrees.");
  Serial.println("Directional Difference:");
  Serial.print(directionDifference);
  Serial.print(" degrees.");
  Serial.print("  Steering Servo ");
  Serial.print(steeringServo);
  Serial.println();
  delay(1000);
  
  
  
  

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
    if (feedgps())
      newdata = true;
  }
  
  if (newdata)
  {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    //Serial.println("-------------");
    //Serial.println((initialLat-myLat)*10000);
  }
}

