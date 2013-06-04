void gpsdump(TinyGPS &gps){
  
  gps.f_get_position(&currentLat, &currentLon, &gpsLockAge);
  currentSpeed = gps.speed() * 0.011;
  currentGPSHeading = gps.course()/100;
  
  if(initialLat==0){
    initialLat = currentLat;
  }
  if(initialLon==0){
    initialLon = currentLon;
  }
  
  
  goalLat = goal[pointid];
  goalLon = goal[pointid+1];
  goalHeading = gps.course_to(currentLat, currentLon, goalLat, goalLon);
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
    turnAngle += integralTuning*turnAngleHistoryAverage;
  
  if(turnAngle > 180){
    turnAngle = 180;
  }
  
  if(turnAngle < -180){
    turnAngle = -180;
  }
  
  if(turnAngle <= 0){
    //steeringAngle = (int) (centerSteeringTuningValue+(fullLeftSteeringTuningValue-centerSteeringTuningValue)*(sin(turnAngle/2)));
    steeringAngle = map(turnAngle, -180, 0, fullLeftSteeringTuningValue, centerSteeringTuningValue);
  }else{
    //steeringAngle = (int) (centerSteeringTuningValue+(centerSteeringTuningValue-fullRightSteeringTuningValue)*(sin(turnAngle/2)));
    steeringAngle = map(turnAngle, 0, 180, centerSteeringTuningValue, fullRightSteeringTuningValue);
  }
  
  distanceToGoal = gps.distance_between(currentLat, currentLon, goalLat, goalLon);
  if(distanceToGoal < goal[pointid+2]){ //check if location is within accuracy range of point
    if(waypointBehavior == 's'){
      if((pointid+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, start course from the first goalpoint
        pointid += 3;
        resetTurnAngleHistory();
      }else{
        goalSpeed = 0;
        drivetrainEnabled = false;
      }
    }else if(waypointBehavior == 'r'){
      if(waypointDirection){
        if((pointid+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, reverse course
          pointid += 3;
          resetTurnAngleHistory();
        }else{
          waypointDirection = false;
          if((pointid-3) > -1){
            pointid -=3;
            resetTurnAngleHistory();
          }
        }
      }else{
        if((pointid-3) > -1){
          pointid -=3;
          resetTurnAngleHistory();
        }else{
          waypointDirection = true;
          if((pointid+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, start course from the first goalpoint
            pointid += 3;
            resetTurnAngleHistory();
          }
        }
      }       
    }else{
      if((pointid+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, start course from the first goalpoint
        pointid += 3;
        resetTurnAngleHistory();
      }else{
        pointid = 0;
        resetTurnAngleHistory();
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
