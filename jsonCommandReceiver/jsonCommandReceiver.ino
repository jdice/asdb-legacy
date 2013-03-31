#include <aJSON.h>
aJsonStream serial_stream(&Serial);
char jsonData[150];
//String parseStr;

void setup(){
  Serial3.begin(57600);
  Serial.begin(9600);
  Serial.println("Running");
}

void loop(){
  if(Serial3.available() > 0){
    int resultSize = Serial3.readBytesUntil('~',jsonData,100);
    if((jsonData[0] == '{') && (jsonData[resultSize-1] == '}')){
      Serial.println(jsonData);
      aJsonObject* jsonObject = aJson.parse(jsonData);
      char *json_String=aJson.print(jsonObject);
      Serial.println(json_String);
      Serial.println(jsonObject->test);
      aJson.deleteItem(jsonObject);
      //parseStr = String(jsonData);
      //Serial.println(parseStr);
      memset(jsonData,0,sizeof(jsonData));
    }else{
      Serial.print("Fail:");
      Serial.println(jsonData);
      memset(jsonData,0,sizeof(jsonData));
    }
  }
}
