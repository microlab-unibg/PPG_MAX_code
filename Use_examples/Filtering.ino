#include <Wire.h>
#include <MAX86916_cm.h>
#include <cmFilter.h>

DigitalFilter filter;
MAX86916 Sensor;
double value;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Initializing...");
  delay(3000);
  Heartrate.init_ecg_algorithm();
  delay(3000);

  // Initialize sensor
  if (!Sensor.begin())
  {
    Serial.println("Sensor not found, check wiring ");
    while (1);
  }

  Sensor.setup(0, 0x1F, MAX86916_SAMPLEAVG_16 , 1, MAX86916_SAMPLERATE_100 , MAX86916_PULSEWIDTH_420, MAX86916_ADCRANGE_4096);
 

}

void loop() {
  // put your main code here, to run repeatedly
  value=Sensor.getIR();
  value=filter.pt1FilterApply(value, &filter.lowpassFilter);
  filter.dcFilter= filter.dcRemoval(value, filter.dcFilter.w, DC_ALPHA);
  value = filter.meanDiffFilter(filter.dcFilter.result, &filter.meanDiff);
  value= filter.lowPass(value);
  Serial.print("Filt:");
  Serial.println(value);
  
  

  

}
