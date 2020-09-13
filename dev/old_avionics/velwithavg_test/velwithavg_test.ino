#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(69);


float accZ;
float accZCal = 0;
int Samps = 25;

float velZ = 0;

float t0;
float deltat;

void calibrate(void)
{
  for (int i=0;i < Samps;i++){
    sensors_event_t event;
    accel.getEvent(&event);
  
    accZ = event.acceleration.z;
    accZCal = accZCal + accZ;
  }
  accZCal = accZCal/Samps;
}


void getAcc(void)
{
  accZ = 0;
  sensors_event_t event;
  for (int i=0;i < Samps;i++)
  {
    accel.getEvent(&event);
    accZ = accZ + event.acceleration.z;
  }
  accZ = accZ/Samps - accZCal;
}

void getVel(void)
{
  velZ = velZ + deltat*accZ;
}

void setup() {
  Serial.begin(230400);
  accel.begin();
  accel.setRange(ADXL345_RANGE_4_G);
  accel.setDataRate(ADXL345_DATARATE_3200_HZ);
  calibrate();
  deltat = 0;
}

void loop() {
  t0 = millis();
  getAcc();
  getVel();
  Serial.println(velZ);
  deltat = millis()-t0;
}
