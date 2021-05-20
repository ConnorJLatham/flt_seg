#include <Wire.h>
#include "SparkFunMPL3115A2.h"

#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>

// CHANGE TO ALLOW FOR VARYING LOOP SIZE

//----------------------------------------------------------------------------
// STATE LOOP DEFINITIONS
//----------------------------------------------------------------------------
unsigned long mission_time;
unsigned long flight_control_loop_delta = 50000;
unsigned long flight_control_loop_time = 0;
unsigned long flight_data_loop_delta = 10000;
unsigned long flight_data_loop_time = 0;
unsigned long flight_tx_loop_delta = 200000;
unsigned long flight_tx_loop_time = 0;


//----------------------------------------------------------------------------
// FLIGHT CONTROL DEFINITIONS
//----------------------------------------------------------------------------
boolean abort_mission = 0;
unsigned int current_state;
unsigned int queued_state;
const String possible_states[11] = {"ground_idle", // ---------- 1
                                    "pad", // ------------------ 2
                                    "abort", // ---------------- 3
                                    "powered_ascent", // ------- 4 
                                    "coast", // ---------------- 5
                                    "arcing_over", // ---------- 6 
                                    "drogue_deploy", // -------- 7
                                    "drogue_descent", // ------- 8
                                    "main_deploy", // ---------- 9
                                    "main_descent", // --------- 10
                                    "landed"}; // -------------- 11


//----------------------------------------------------------------------------
// FLIGHT DATA DEFINITIONS
//----------------------------------------------------------------------------

const float deg_to_rad = PI/180;
const int cal_reading_count = 1000;

float gyro_x, gyro_y, gyro_z;
float gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;

float q [4] = {1, 0, 0, 0};
float omega [16] = {};
float q_dot [4] = {0, 0, 0, 0};


//----------------------------------------------------------------------------
// HARDWARE DEFINITIONS
//----------------------------------------------------------------------------

int pin_green = 2, pin_red = 3, pin_blue = 4, pin_yellow = 5;
LSM9DS1 imu;


//----------------------------------------------------------------------------
// BEGIN SETUP
//----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200); // start serial at a high baud rate
  Wire.begin(); // turn on i2c

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = 0x1E;
  imu.settings.device.agAddress = 0x6B;
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
  imu.setGyroScale(2000);
  
  queued_state = "ground_idle";
  current_state = 1;
  setup_sensors();
}


//----------------------------------------------------------------------------
// BEGIN LOOP
//----------------------------------------------------------------------------
void loop() {

  // get the mission time at the front of the loop for comparison in the switch
  mission_time = micros();
  
  
  //----------------------------------------------------------------------------
  // BEGIN SWITCH
  //----------------------------------------------------------------------------
  switch(current_state) {
    
    
    //----------------------------------------------------------------------------
    // GROUND STATE
    //----------------------------------------------------------------------------
    case 1:
      // run the data loop to collect most recent flight data
      if (mission_time-flight_data_loop_time >= flight_data_loop_delta) {
//        Serial.print("Data loop: ");
        Serial.println(mission_time-flight_data_loop_time);
        flight_data_loop_time = mission_time;
        collect_sensor_data(flight_data_loop_time);
        calc_omega();
        calc_quart_dot();
        calc_orientation();
        print_quats();
//        print_gyro();
      }
    
    //----------------------------------------------------------------------------
    // UNDEFINED
    //----------------------------------------------------------------------------
  } 
}


//----------------------------------------------------------------------------
// FUNCTION DEFINITIONS
//----------------------------------------------------------------------------
void setup_sensors()
{
  calibrate_sensors();
}

void collect_sensor_data(long collection_time)
{



  imu.readGyro();
  gyro_x = (imu.calcGyro(imu.gx)-gyro_x_cal)*deg_to_rad;
  gyro_y = (imu.calcGyro(imu.gy)-gyro_y_cal)*deg_to_rad;
  gyro_z = (imu.calcGyro(imu.gz)-gyro_z_cal)*deg_to_rad;

  

  // if data_counter is less than the maximum, add data to the arrays
}

void calibrate_sensors() {
  for (int i = 0; i < cal_reading_count; i++) {
    delay(5);
    
   
    imu.readGyro();
    
    

    gyro_x_cal += imu.calcGyro(imu.gx);
    gyro_y_cal += imu.calcGyro(imu.gy);
    gyro_z_cal += imu.calcGyro(imu.gz);
  }
  

  gyro_x_cal = gyro_x_cal/float(cal_reading_count);
  gyro_y_cal = gyro_y_cal/float(cal_reading_count);
  gyro_z_cal = gyro_z_cal/float(cal_reading_count);
}

void calc_orientation()
{
  float delta = flight_data_loop_delta/1000000.0;
  q[0] += q_dot[0]*delta;
  q[1] += q_dot[1]*delta;
  q[2] += q_dot[2]*delta;
  q[3] += q_dot[3]*delta;

  float q_norm = sqrt(sqr(q[0])+sqr(q[1])+sqr(q[2])+sqr(q[3]));
  
  q[0] = q[0]/q_norm;
  q[1] = q[1]/q_norm;
  q[2] = q[2]/q_norm;
  q[3] = q[3]/q_norm;  
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

void calc_quart_dot()
{
  q_dot[0] = -.5*float(omega[0]*q[0] + omega[1]*q[1] + omega[2]*q[2] + omega[3]*q[3]);
  q_dot[1] = -.5*float(omega[4]*q[0] + omega[5]*q[1] + omega[6]*q[2] + omega[7]*q[3]);
  q_dot[2] = -.5*float(omega[8]*q[0] + omega[9]*q[1] + omega[10]*q[2] + omega[11]*q[3]);
  q_dot[3] = -.5*float(omega[12]*q[0] + omega[13]*q[1] + omega[14]*q[2] + omega[15]*q[3]);
}


float sqr(float num)
{
  return num*num;
}

void print_quats()
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

void print_gyro() {
  Serial.print(gyro_x);
  Serial.print(", ");
  Serial.print(gyro_y);
  Serial.print(", ");
  Serial.println(gyro_z);
}

