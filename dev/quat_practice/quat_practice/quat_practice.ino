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
unsigned long flight_control_loop_delta = 500;
unsigned long flight_control_loop_time = 0;
unsigned long flight_data_loop_delta = 1500;
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
const int cal_reading_count = 250;

float gyro_x, gyro_y, gyro_z;
float gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;

float quat_w = 1.0, quat_i = 0.0, quat_j = 0.0, quat_k = 0.0;
float qw_dot = 0.0, qi_dot = 0.0, qj_dot = 0.0, qk_dot = 0.0;


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
        Serial.print("Data loop: ");
        Serial.println(mission_time-flight_data_loop_time);
        flight_data_loop_time = mission_time;
        collect_sensor_data(flight_data_loop_time);
        calc_quat();
        prop_quat();

      }
      
//      // run the control loop to calculate various flight data
//      if (mission_time-flight_control_loop_time >= flight_control_loop_delta) {
//        Serial.print("Control loop: ");
//        Serial.println(mission_time-flight_control_loop_time);
//        flight_control_loop_time = mission_time;
//
//        calc_quat();
//      }
    
//      // run the tx loop to transmit flight data
//      if (mission_time-flight_tx_loop_time >= flight_tx_loop_delta) {
//        Serial.print("Tx Loop: ");
//        Serial.println(mission_time-flight_tx_loop_time);
//        flight_tx_loop_time = mission_time;
//      }
      
    
    
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

// collect sensor data and append to the array
// keep track of when we collect this data as well
void collect_sensor_data(long collection_time)
{



  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }



  // get gyro rates in rad/s
  gyro_x = (imu.calcGyro(imu.gx)-gyro_x_cal)*deg_to_rad;
  gyro_y = (imu.calcGyro(imu.gy)-gyro_y_cal)*deg_to_rad;
  gyro_z = (imu.calcGyro(imu.gz)-gyro_z_cal)*deg_to_rad;

  

  // if data_counter is less than the maximum, add data to the arrays
}

void calibrate_sensors() {
  for (int i = 0; i < cal_reading_count; i++) {
    delay(10);
    
   
    imu.readGyro();
    
    

    gyro_x_cal += imu.calcGyro(imu.gx);
    gyro_y_cal += imu.calcGyro(imu.gy);
    gyro_z_cal += imu.calcGyro(imu.gz);
  }
  

  gyro_x_cal = gyro_x_cal/cal_reading_count;
  gyro_y_cal = gyro_y_cal/cal_reading_count;
  gyro_z_cal = gyro_z_cal/cal_reading_count;
}

void calc_quat() {
  
//  qi_dot = 0.5*(gyro_z*quat_j - gyro_y*quat_k + gyro_x*quat_w);
//  qj_dot = 0.5*(-gyro_z*quat_i + gyro_x*quat_k + gyro_y*quat_w);
//  qk_dot = 0.5*(gyro_y*quat_i - gyro_x*quat_j + gyro_z*quat_w);
//  qw_dot = 0.5*(-gyro_x*quat_i - gyro_y*quat_j - gyro_z*quat_k);

  qi_dot = 0.5*(quat_w*gyro_x - quat_k*gyro_y + quat_j*gyro_z);
  qj_dot = 0.5*(quat_k*gyro_x + quat_w*gyro_y - quat_i*gyro_z);
  qk_dot = 0.5*(-quat_j*gyro_x + quat_i*gyro_y + quat_w*gyro_z);
  qw_dot = 0.5*(-quat_i*gyro_x - quat_j*gyro_y - quat_k*gyro_z);
  
//  Serial.println("Quat dots");
//  Serial.print(qw_dot);
//  Serial.print(" ");
//  Serial.print(qi_dot);
//  Serial.print(" ");
//  Serial.print(qj_dot);
//  Serial.print(" ");
//  Serial.print(qk_dot);
//  Serial.println(" ");
}

void prop_quat() {
  
  float delta = flight_data_loop_delta/1000000.0;
  quat_w += qw_dot*delta;
  quat_i += qi_dot*delta;
  quat_j += qj_dot*delta;
  quat_k += qk_dot*delta;
  
  float quat_mag = sqrt(quat_w*quat_w + quat_i*quat_i + quat_j*quat_j + quat_k*quat_k);
  quat_w = quat_w/quat_mag;
  quat_i = quat_i/quat_mag;
  quat_j = quat_j/quat_mag;
  quat_k = quat_k/quat_mag;
  
  Serial.println("Quats");
  Serial.print(quat_w);
  Serial.print(" ");
  Serial.print(quat_i);
  Serial.print(" ");
  Serial.print(quat_j);
  Serial.print(" ");
  Serial.print(quat_k);
  Serial.println(" ");
}

