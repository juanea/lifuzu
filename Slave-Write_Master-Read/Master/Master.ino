//i2c Master(UNO)
#include <Wire.h>

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  
  Wire.requestFrom(5,10);
  
  while(Wire.available())
  {
    char c = Wire.read();
    Serial.print(c);
  }
}

void loop()
{
}
