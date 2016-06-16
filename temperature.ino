#include <Wire.h>

#define address 0x48
#define delayC 1000
#define baudrate 9600

void setup()
{
  Wire.begin();
  Serial.begin(baudrate);
}

void loop()
{
  Serial.print("temperature in Celsuis: ");
  int temperature;
  Wire.beginTransmission(address);

  Wire.write(0x00);

  Wire.requestFrom(address, 1);
  if (Wire.available())
  {
    temperature = Wire.read();
    Serial.println(temperature);
  }
  else
  {
    Serial.println("---");
  }
  Wire.endTransmission();
  delay(delayC);
}

