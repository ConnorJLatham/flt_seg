#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(69);


double accZ;
double accX;
double accY;
double accZCal = 0;
int calSamps = 10;
double accMag;

double velZ = 0;

double altZ = 0;

double t0;
double tf;
double deltat;

void calibrate(void)
{
  for (int i=0;i < 10;i++){
    sensors_event_t event;
    accel.getEvent(&event);
  
    accZ = event.acceleration.z;
    accZCal = accZCal + accZ;
  }
  accZCal = accZCal/calSamps;
}

void getAlt(void)
{
  altZ = altZ + deltat*velZ;
}
void getVel(void)
{
  velZ = velZ + deltat*accMag;
}
void getAcc(void)
{
  sensors_event_t event;
  accel.getEvent(&event);

  accX = event.acceleration.x;
  accY = event.acceleration.y;
  accZ = event.acceleration.z;
  accMag = sqrt(pow(accX,2)+pow(accY,2)+pow(accZ,2))-accZCal;
}

void setup() {
  t0 = millis();
  Serial.begin(230400);
  accel.begin();
  accel.setRange(ADXL345_RANGE_4_G);
  calibrate();
  tf = millis();
  deltat = (tf-t0)/1000;
}

void loop() {
  t0 = millis();
  getAcc();
  getVel();
  getAlt();
  tf = millis();
  deltat = (tf-t0)/1000;
  Serial.println(altZ);
}
