#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(69);


float accZ;
float accZCal = 0;
int Samps = 15;
float ati;
float ati1;
double ti;
double ti1;
double accTime;
double velTime;

float velZ = 0;

float altZ = 0;

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


float getAcc(void)
{
  accZ = 0;
  sensors_event_t event;
  for (int i=0;i < Samps;i++)
  {
    accel.getEvent(&event);
    if (i == Samps/2)
    {
      accTime = millis();
    }
    accZ = accZ + event.acceleration.z;
  }
  accZ = accZ/Samps - accZCal;
  return accZ;
}

void getInstVel(void)
{
  velTime = millis();
  velZ = ((ati+ati1)/2)*(ti1-ti)/1000;
}

void getAlt(void)
{
  altZ = altZ + velZ*(ti1-ti)/1000;
}

void setup() {
  Serial.begin(230400);
  accel.begin();
  accel.setRange(ADXL345_RANGE_4_G);
  accel.setDataRate(ADXL345_DATARATE_3200_HZ);
  calibrate();
  ati = getAcc();
  ti = accTime;
  
}

void loop() {
  ati1 = getAcc();
  ti1 = accTime;
  getInstVel();
  getAlt();
  Serial.println(altZ);
  ati = ati1;
  ti = ti1;
}
