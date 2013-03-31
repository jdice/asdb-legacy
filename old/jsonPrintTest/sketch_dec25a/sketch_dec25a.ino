double currentLat, currentLon, goalLat, goalLon, currentHeading, goalHeading, currentSpeed, goalSpeed, maxSpeed, imuYaw, imuPitch, imuRoll, imuAccX, imuAccY, imuAccZ = 0.00;
boolean steeringEnabled, drivetrainEnabled, headlightEnabled, adjustMySpeed, gpsLock, leftBumper, rightBumper = true;
void setup()
{
  Serial.begin(115200);
}

void printJSON(){
  Serial.print("{\n\t\"status\":{\n\t\t\"steeringEnabled\":");
  printBoolean(steeringEnabled);
  Serial.print(",\n\t\t\"drivetrainEnabled\":");
  printBoolean(drivetrainEnabled);
  Serial.print(",\n\t\t\"headlightEnabled\":");
  printBoolean(headlightEnabled);
  Serial.print(",\n\t\t\"adjustMySpeed\":");
  printBoolean(adjustMySpeed);
  Serial.print(",\n\t\t\"gpsLock\":");
  printBoolean(gpsLock);
  Serial.print(",\n\t\t\"leftBumper\":");
  printBoolean(leftBumper);
  Serial.print(",\n\t\t\"rightBumper\":");
  printBoolean(rightBumper);
  Serial.print("\n\t},\n\t\"location\":{\n\t\t\"current\":{\n\t\t\t\"lat\":");
  printFloat(currentLat, 5);
  Serial.print(",\n\t\t\t\"lon\":");
  printFloat(currentLon, 5);
  Serial.print("\n\t\t},\n\t\t\"goal\":{\n\t\t\t\"lat\":");
  printFloat(goalLat, 5);
  Serial.print(",\n\t\t\t\"lon\":");
  printFloat(goalLon, 5);
  Serial.print("\n\t\t},\n\t\t\"heading\":{\n\t\t\t\"current\":");
  printFloat(currentHeading, 5);
  Serial.print(",\n\t\t\t\"goal\":");
  printFloat(goalHeading, 5);
  Serial.print("\n\t\t},\n\t\t\"speed\":{\n\t\t\t\"current\":");
  printFloat(currentSpeed, 5);
  Serial.print(",\n\t\t\t\"goal\":");
  printFloat(goalSpeed, 5);
  Serial.print(",\n\t\t\t\"max\":");
  printFloat(maxSpeed, 5);
  Serial.print("\n\t\t},\n\t\t\"imu\":{\n\t\t\t\"yaw\":");
  printFloat(imuYaw, 5);
  Serial.print(",\n\t\t\t\"pitch\":");
  printFloat(imuPitch, 5);
  Serial.print(",\n\t\t\t\"roll\":");
  printFloat(imuRoll, 5);
  Serial.print(",\n\t\t\t\"accX\":");
  printFloat(imuAccX, 5);
  Serial.print(",\n\t\t\t\"accY\":");
  printFloat(imuAccY, 5);
  Serial.print(",\n\t\t\t\"accZ\":");
  printFloat(imuAccZ, 5);
  Serial.print("\n\t\t}\n\t},\n\t\"debug\":");
  Serial.print("\"Debug info.\"");
  Serial.print("\n}~");
}

void printBoolean(boolean value){
  if(value){
    Serial.print("true");
  }else{
    Serial.print("false");
  }
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

void loop()
{
  currentLat = rand();
  currentLon = rand();
  printJSON();
  delay(1000);
}
