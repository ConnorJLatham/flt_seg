#include <Wire.h>
#include "SparkFunMPL3115A2.h"

#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>



// angular velocities
float gyro_x;
float gyro_y;
float gyro_z;
int read_count = 1;

// angular velocities calibrations
float gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;
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

LSM9DS1 imu;

//Function definitions

void setup() 
{
  
  Serial.begin(115200);
  Wire.begin(); // turn on i2c
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = 0x1E;
  imu.settings.device.agAddress = 0x6B;
  imu.begin();
  // Use setSensors to turn on or off MPU-9250 sensors.
  // Any of the following defines can be combined:
  // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
  // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  // Enable all sensors:
//  imu.setSensors(INV_XYZ_GYRO);
//  // Use setGyroFSR() and setAccelFSR() to configure the
//  // gyroscope and accelerometer full scale ranges.
//  // Gyro options are +/- 250, 500, 1000, or 2000 dps
//  imu.setGyroFSR(2000); // Set gyro to 2000 dps
//  // setLPF() can be used to set the digital low-pass filter
//  // of the accelerometer and gyroscope.
//  // Can be any of the following: 188, 98, 42, 20, 10, 5
//  // (values are in Hz).
//  imu.setLPF(188); // Set LPF corner frequency to 5Hz
//  // The sample rate of the accel/gyro can be set using
//  // setSampleRate. Acceptable values range from 4Hz to 1kHz
//  imu.setSampleRate(1000); // Set sample rate to 10Hz

  imu.setGyroScale(2000);
  
  gyro_calibration();
}

void loop()
{
  t0 = t1;
  t1 = millis()/float(1000);
  read_gyros();
  calc_omega();
  calc_quart_dot();
  calc_orientation();
//  quart_to_euler();
  print_quats(q);
  
  
}

void read_gyros()
{
  imu.readGyro();
  gyro_x = deg_2_rad(imu.calcGyro(imu.gx-gyro_x_cal));
  gyro_y = deg_2_rad(imu.calcGyro(imu.gy-gyro_y_cal));
  gyro_z = deg_2_rad(imu.calcGyro(imu.gz-gyro_z_cal));
  print_gyros(gyro_x, gyro_y, gyro_z);
}

void print_gyros(float gxx, float gyy, float gzz)
{
  Serial.print(gxx);
  Serial.print(", ");
  Serial.print(gyy);
  Serial.print(", ");
  Serial.print(gzz);
  Serial.println();
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

void quart_to_euler()
{
  theta_x = atan2(2*(q[2]*q[3]+q[0]*q[1]),1-2*(sqr(q[1])+sqr(q[2])))*180/PI;
  theta_y = asin(-q[1]*q[3] + q[0]*q[2])*180/PI;
  theta_z = atan2(2*(q[1]*q[2]+q[0]*q[3]),1-2*(sqr(q[2])+sqr(q[3])))*180/PI;
}

void gyro_calibration()
{
  for(int i=0 ; i < cal_count ; i++) 
  {
    imu.readGyro();
    gyro_x_cal += imu.calcGyro(imu.gx);
    gyro_y_cal += imu.calcGyro(imu.gy);
    gyro_z_cal += imu.calcGyro(imu.gz);
  }
  gyro_x_cal = gyro_x_cal / float(cal_count);
  gyro_y_cal = gyro_y_cal / float(cal_count);
  gyro_z_cal = gyro_z_cal / float(cal_count);
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

//void print_angles(float phi, float theta, float psi)
//{
//  Serial.print(phi);
//  Serial.print(", ");
//  Serial.print(theta);
//  Serial.print(", ");
//  Serial.print(psi);
//  Serial.println();
//}

