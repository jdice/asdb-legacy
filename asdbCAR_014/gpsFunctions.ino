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
  
  
  goalLat = goal[pointId];
  goalLon = goal[pointId+1];
  goalHeading = gps.course_to(currentLat, currentLon, goalLat, goalLon);
  
  
  distanceToGoal = gps.distance_between(currentLat, currentLon, goalLat, goalLon);
  if(distanceToGoal < goal[pointId+2]){ //check if location is within accuracy range of point
    if(waypointBehavior == 's'){
      if((pointId+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, start course from the first goalpoint
        pointId += 3;
        resetTurnAngleHistory();
      }else{
        steeringEnabled = false;
        buttonStop = true; // need button to start again
      }
    }else if(waypointBehavior == 'r'){
      if(waypointDirection){
        if((pointId+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, reverse course
          pointId += 3;
          resetTurnAngleHistory();
        }else{
          waypointDirection = false;
          if((pointId-3) > -1){
            pointId -=3;
            resetTurnAngleHistory();
          }
        }
      }else{
        if((pointId-3) > -1){
          pointId -=3;
          resetTurnAngleHistory();
        }else{
          waypointDirection = true;
          if((pointId+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, start course from the first goalpoint
            pointId += 3;
            resetTurnAngleHistory();
          }
        }
      }       
    }else{
      // Standard loop behavior
      if((pointId+3) < (sizeof(goal)/4)){ // check to see if there is a "next" point, if not, start course from the first goalpoint
        pointId += 3;
        resetTurnAngleHistory();
      }else{
        pointId = 0;
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
