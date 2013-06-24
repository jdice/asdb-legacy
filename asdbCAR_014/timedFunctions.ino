void timedFunctions(){
  if(millis() - timer100 > 100){
    // Update max speed
    if((currentSpeed > maxSpeed) && (currentSpeed < 10000)){
      maxSpeed = currentSpeed;
    }
    
    // Update headlight
    /*if(headlightEnabled){
      digitalWrite(headlightPin, HIGH);
    }else{
      digitalWrite(headlightPin, LOW);
    }*/
    
    // Send JSON data
    if(jsonPrint){
      aJsonObject *msg = createMessage();
      aJson.print(msg, &serial_stream);
      //Serial3.print("~");
      //Serial3.println();
      Serial.print("~");
      Serial.println();
      aJson.deleteItem(msg);
    }
    loopNumber = 0;
    timer100 = millis();
  }
  
  if(millis() - timer500 > 500){
    //adjustSpeed();
    timer500 = millis();
  }
}
