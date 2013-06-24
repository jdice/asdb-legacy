void adjustSpeed(){
  /*if(adjustMySpeed){
    if( (currentSpeed > (goalSpeed + speedTolerance)) && (speedValue < 85) ){
      speedValue++;
    }
    if( (currentSpeed < (goalSpeed - speedTolerance)) && (speedValue > 65) ){
      speedValue--;
    }
  }*/
}

void writeSpeed(){
  //stop criteria
  if(!stopOverrides){
    if(buttonStop || xbeeStop || bumperStop){ // all optional stopping conditions except rangefinder
      if(drivetrainEnabled){
        brakeMode = true;
        brakeTimer = millis()+brakeDuration;
      }
      drivetrainEnabled = false;
    }else{ // if NONE of the stopping conditions are met, enable drivetrain
      drivetrainEnabled = true;
    }
  }else{
    if(buttonStop || xbeeStop){ // all absolute stopping conditions
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
    if(adjustMySpeed == false){
      if(slowMode){
        speedValue = slowSpeedValue;
      }else{
        if(distanceToGoal < slowLimit){
          speedValue = slowSpeedValue;
        }else if(distanceToGoal < fastLimit){
          speedValue = map(distanceToGoal, slowLimit, fastLimit, slowSpeedValue, fastSpeedValue);
        }else{
          speedValue = fastSpeedValue;
        }
      }
      adjustMySpeed = false;
    }
    adjustedSpeedValue = speedValue;
  }else{
    adjustMySpeed = false;
    if(millis() > brakeTimer){
      brakeMode = false;
    }
    if(brakeMode){
      adjustedSpeedValue = brakeSpeedValue; // brake the car for brakeDuration
    }else{
      adjustedSpeedValue = neutralSpeedValue; // stop the car if drivetrain is disabled.
    }
  }
  speedControl.write(adjustedSpeedValue);
}

void steeringControl(){
  if(steeringEnabled){
      steeringServo.write(steeringAngle);
  }
}
