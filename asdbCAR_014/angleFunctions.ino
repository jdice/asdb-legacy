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

void resetTurnAngleHistory(){
  for(int i = 0; i < turnAngleHistorySize; i++){
    turnAngleHistory[i] = 0;
  }
}
