/*
 * Author: Krutav Shah
 * Notes:  4 Time Of Flight sensor program for arduino nano and i2c multiplexer
 */

#include "Wire.h"
#include "VL6180X.h"
#define TCAADDR 0x70

VL6180X sd0;
VL6180X sd1;
VL6180X sd2;
VL6180X sd3;

int d[4] = {0,0,0,0};

void tcaselect (uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void initSensor (VL6180X sensor){
  sensor.init();
  sensor.configureDefault();

  // Reduce range max convergence time and ALS integration
  // time to 30 ms and 50 ms, respectively, to allow 10 Hz
  // operation (as suggested by Table 6 ("Interleaved mode
  // limits (10 Hz operation)") in the datasheet).
  sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor.setTimeout(0);

  // Stop continuous mode if already active
  sensor.stopContinuous();
  // in case stopContinuous() triggered a single-shot
  // measurement, wait for it to complete
  delay(100);
  // start interleaved continuous mode with period of 100 ms
  sensor.startInterleavedContinuous(100);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  //Indexer chamber0 sensor
  tcaselect(0);
  initSensor(sd0);

  //Indexer chamber1 sensor
  tcaselect(1);  
  initSensor(sd1);

  //Indexer chamber2 sensor
  tcaselect(2);  
  initSensor(sd2);

  //Tunnel sensor
  tcaselect(3);
  initSensor(sd3);

}
  
void loop() 
{ 
  tcaselect(0); 
  d[0] = sd0.readRangeContinuousMillimeters(); 
  Serial.println((String)"0." + d[0]);
  
  tcaselect(1); 
  d[1] = sd1.readRangeContinuousMillimeters();
  Serial.println((String)"1." + d[1]); 
  
  tcaselect(2); 
  d[2] = sd2.readRangeContinuousMillimeters(); 
  Serial.println((String)"2." + d[2]);
  
  tcaselect(3); 
  d[3] = sd3.readRangeContinuousMillimeters();
  Serial.println((String)"3." + d[3]);
}
