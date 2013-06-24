void readSensors(){
  // Update sensor readings
  
  // Read from XBee
  while(Serial.available()){  //is there anything to read?
    char getData = Serial.read();  //if yes, read it
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
  
  // Start Button
  if(buttonStop == true){
    if(digitalRead(startButtonPin) == LOW){
      buttonStop = false;
      pointId = 0;
      steeringEnabled = true;
    }
  }
  
  // Bumpers
  if(digitalRead(leftBumperPin) == HIGH){
    leftBumper = false;
  }else{
    leftBumper = false;
  }
  if(digitalRead(rightBumperPin) == HIGH){
    rightBumper = false; // DO NOT LISTEN TO THE RIGHT BUMPER BECAUSE IT IS BROKEN AND REMEMBER TO FIX THIS LATER PLEASE AND THANK YOU
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
  if(rangefinderValue < 80){
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
  
  // Get new steering angle
  adjustGoalHeading();
  turnAngle = getTurnAngle(adjustedGoalHeading, currentHeading);
  if(loopNumber == 1){
    for(int i = 1; i <= turnAngleHistorySize; i++){
      turnAngleHistory[i] = turnAngleHistory[i-1];
    }
    turnAngleHistory[0] = turnAngle;
    turnAngleHistoryAverage = 0;
    for(int i = 0; i < turnAngleHistorySize; i++){
      turnAngleHistoryAverage += turnAngleHistory[i];
    }
    turnAngleHistoryAverage /= turnAngleHistorySize;
  }
    //turnAngle += integralTuning*turnAngleHistoryAverage;
  
  if(turnAngle > 180){
    turnAngle = 180;
  }
  
  if(turnAngle < -180){
    turnAngle = -180;
  }
  
  if(turnAngle <= 0){
    //steeringAngle = (int) (centerSteeringTuningValue+(fullLeftSteeringTuningValue-centerSteeringTuningValue)*(sin(turnAngle*PI/360)));
    steeringAngle = map(turnAngle, -180, 0, fullLeftSteeringTuningValue, centerSteeringTuningValue);
  }else{
    //steeringAngle = (int) (centerSteeringTuningValue+(centerSteeringTuningValue-fullRightSteeringTuningValue)*(sin(turnAngle*PI/360)));
    steeringAngle = map(turnAngle, 0, 180, centerSteeringTuningValue, fullRightSteeringTuningValue);
  }
  
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
