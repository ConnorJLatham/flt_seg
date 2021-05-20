#include <MPU9250.h>
#include <quaternionFilters.h>
#include <Wire.h>

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

// angular velocities
float gyro_x;
float gyro_y;
float gyro_z;
int read_count = 1;

// angular velocities calibrations
float gyro_cal_x;
float gyro_cal_y;
float gyro_cal_z;
int cal_count = 1000;

// angular orientations
float theta_x = 0;
float theta_y = 0;
float theta_z = 0;

// quaternions
float q_norm;
float q [4] = {1, 0, 0, 0}; 
float q_dot [4] = {0, 0, 0, 0};
float omega [16] = {};

// timekeeping
float t0 = 0;
float t1 = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initMPU9250();
  myIMU.getGres();
  gyro_calibration();
}

void loop() {
  t0 = t1;
  t1 = millis()/float(1000);
  read_gyros();
  calc_omega();
  calc_quart_dot();
  calc_orientation();
  print_quats(q);
  

  

//  Serial.print(myIMU.gx);Serial.print(", "); Serial.print(myIMU.gy); Serial.print(", "); Serial.println(myIMU.gz);
}

void read_gyros()
{
  myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
  // Calculate the gyro value into actual degrees per second
  // This depends on scale being set
  gyro_x = deg_2_rad((float)myIMU.gyroCount[0] * myIMU.gRes - gyro_cal_x);
  gyro_y = deg_2_rad((float)myIMU.gyroCount[1] * myIMU.gRes - gyro_cal_y);
  gyro_z = deg_2_rad((float)myIMU.gyroCount[2] * myIMU.gRes - gyro_cal_z);
}

void calc_quart_dot()
{
  q_dot[0] = -.5*float(omega[0]*q[0] + omega[1]*q[1] + omega[2]*q[2] + omega[3]*q[3]);
  q_dot[1] = -.5*float(omega[4]*q[0] + omega[5]*q[1] + omega[6]*q[2] + omega[7]*q[3]);
  q_dot[2] = -.5*float(omega[8]*q[0] + omega[9]*q[1] + omega[10]*q[2] + omega[11]*q[3]);
  q_dot[3] = -.5*float(omega[12]*q[0] + omega[13]*q[1] + omega[14]*q[2] + omega[15]*q[3]);
}

void calc_omega()
{
  omega [0] = 0;
  omega [1] = gyro_x;
  omega [2] = gyro_y;
  omega [3] = gyro_z;
  omega [4] = -gyro_x;
  omega [5] = 0;
  omega [6] = -gyro_z;
  omega [7] = gyro_y;
  omega [8] = -gyro_y;
  omega [9] = gyro_z;
  omega [10] = 0;
  omega [11] = -gyro_x;
  omega [12] = -gyro_z;
  omega [13] = -gyro_y;
  omega [14] = gyro_x;
  omega [15] = 0;
}

void calc_orientation()
{
  q[0] += q_dot[0]*(t1-t0);
  q[1] += q_dot[1]*(t1-t0);
  q[2] += q_dot[2]*(t1-t0);
  q[3] += q_dot[3]*(t1-t0);

  q_norm = sqrt(sqr(q[0])+sqr(q[1])+sqr(q[2])+sqr(q[3]));
  
  q[0] = q[0]/q_norm;
  q[1] = q[1]/q_norm;
  q[2] = q[2]/q_norm;
  q[3] = q[3]/q_norm;  
}

void gyro_calibration()
{
  for(int i=0 ; i < cal_count ; i++) 
  {
    myIMU.readGyroData(myIMU.gyroCount);
    gyro_cal_x = gyro_cal_x + (float)myIMU.gyroCount[0];
    gyro_cal_y = gyro_cal_y + (float)myIMU.gyroCount[1];
    gyro_cal_z = gyro_cal_z + (float)myIMU.gyroCount[2];
  }
  gyro_cal_x = gyro_cal_x*myIMU.gRes / float(cal_count);
  gyro_cal_y = gyro_cal_y*myIMU.gRes / float(cal_count);
  gyro_cal_z = gyro_cal_z*myIMU.gRes / float(cal_count);
}

float sqr(float num)
{
  return num*num;
}

float deg_2_rad(float num)
{
  return num*float(PI/180);
}

void print_quats(float quat[4])
{
  Serial.print(q[0]);
  Serial.print(", ");
  Serial.print(q[1]);
  Serial.print(", ");
  Serial.print(q[2]);
  Serial.print(", ");
  Serial.print(q[3]);
  Serial.println();
}
