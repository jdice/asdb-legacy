void readSensors(){
  // Update sensor readings
  
  // Read from XBee
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
    }else if(getData == 'o'){
      slowSpeedValue++;
    }else if(getData == 'p'){
      slowSpeedValue--;
    }
  }
  
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
  
  // Magnetometer
  MagnetometerScaled magScaled = compass.ReadScaledAxis();
  currentHeading = (atan2(magScaled.YAxis, magScaled.XAxis) - declinationAngle)*(180/PI);
  
  while(currentHeading > 360){
    currentHeading -= 360;
  }
  while(currentHeading < 0){
    currentHeading += 360;
  }
  Serial.println(currentHeading);
  
  // Drive Battery Monitor
  driveBatteryVoltage = analogRead(driveBatteryMonitorPin) * 0.00418 * driveBatteryVoltageRatio;
  
  // GPS
  if(feedgps()){
    digitalWrite(lockLED, HIGH);
    gpsLock = true;
    gpsdump(gps);
  }else{
    digitalWrite(lockLED, LOW);
    gpsLock = false;
  }
}
