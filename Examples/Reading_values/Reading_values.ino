#include <Wire.h>
#include <MAX86916_cm.h>

MAX86916 Sensor;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial monitor ready to read current values");

  //establish i2c connection
  if (Sensor.begin()==false)
  {
    Serial.println("Sensor not found. Check wiring");
    while(true);
  }

  Sensor.setup(); //default setup 
  //default setup is range=0, powerleve =0x1F, sampleAverage=4, ledMode=3, sampleRate=400, pulseWidth= 420, adcRange=16k

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("IR:");
  Serial.print(Sensor.getIR());
  Serial.print(" R:");
  Serial.print(Sensor.getRed());
  Serial.print(" G:");
  Serial.print(Sensor.getGreen());
  Serial.print(" B:");
  Serial.print(Sensor.getBlue());

  Serial.println();

}
