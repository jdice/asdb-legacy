void loop(){
  loopNumber++;
  
  readSensors();
  
  writeSpeed();
  
  steeringControl();
  
  timedFunctions();
}
