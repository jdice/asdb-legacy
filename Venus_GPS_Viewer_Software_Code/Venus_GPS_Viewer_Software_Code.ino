//#include (SoftwareSerial.h) // change the parens to angle-brackets

//SoftwareSerial GPS(2,3); // 2 is RX, 3 is TX

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);
}

void loop()
{
  if(Serial2.available())
    Serial.write(Serial2.read());

  if(Serial.available())
    Serial2.write(Serial.read());
}
