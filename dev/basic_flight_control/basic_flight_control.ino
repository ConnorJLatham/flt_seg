#include <Wire.h>
#include "SparkFunMPL3115A2.h"

#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>



//----------------------------------------------------------------------------
// STATE LOOP DEFINITIONS
//----------------------------------------------------------------------------
unsigned long mission_time;
unsigned long flight_control_loop_delta = 200000;
unsigned long flight_control_loop_time = 0;
unsigned long flight_data_loop_delta = 50000;
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
unsigned int data_counter = 0;
const unsigned int max_data_count = 100;
unsigned long data_time_arr[max_data_count];
const float deg_to_rad = PI/180;
const int cal_reading_count = 250;

float accel_x, accel_y, accel_z;
float accel_x_cal = 0, accel_y_cal = 0, accel_z_cal = 0;
float accel_arr[max_data_count][3];

float vel_x = 0.0, vel_y = 0.0, vel_z = 0.0;
float pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
float lat, lon;

float altitude;
float altitude_arr[max_data_count];

float gyro_x, gyro_y, gyro_z;
float gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;
float gyro_arr[max_data_count][3];

float mag_x, mag_y, mag_z;
float mag_arr[max_data_count][3];

float quat_w = 1.0, quat_i = 0.0, quat_j = 0.0, quat_k = 0.0;


//----------------------------------------------------------------------------
// HARDWARE DEFINITIONS
//----------------------------------------------------------------------------

int pin_green = 2, pin_red = 3, pin_blue = 4, pin_yellow = 5;
MPL3115A2 altimeter;
LSM9DS1 imu;
// SDO_XM and SDO_G are both pulled high, so our addresses are:
//#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
//#define LSM9DS1_AG 0x6B // Would be 0x6A if SDO_AG is LOW


//----------------------------------------------------------------------------
// BEGIN SETUP
//----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200); // start serial at a high baud rate
  Wire.begin(); // turn on i2c
  altimeter.begin(); // start the altimeter
  altimeter.setModeAltimeter(); // measure altitude above sea level in meters
  altimeter.setOversampleRate(7);
  altimeter.enableEventFlags();

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
  
  queued_state = "ground_idle";
  current_state = 1;
  setup_sensors();
  pinMode(pin_green, OUTPUT);
  pinMode(pin_red, OUTPUT);
  pinMode(pin_blue, OUTPUT);
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

        data_counter += 1;
      }
      
      // run the control loop to calculate various flight data
      if (mission_time-flight_control_loop_time >= flight_control_loop_delta) {
        Serial.print("Control loop: ");
        Serial.println(mission_time-flight_control_loop_time);
        flight_control_loop_time = mission_time;
      }
    
      // run the tx loop to transmit flight data
      if (mission_time-flight_tx_loop_time >= flight_tx_loop_delta) {
        Serial.print("Tx Loop: ");
        Serial.println(mission_time-flight_tx_loop_time);
        flight_tx_loop_time = mission_time;
      }
      if (data_counter >= max_data_count) {
        current_state = 2;
      }
      break;
    
    
    //----------------------------------------------------------------------------
    // FLIGHT STATE
    //----------------------------------------------------------------------------
    case 2:
      for (int i = 0; i < max_data_count; i++) {
        Serial.println(accel_arr[i][2]);  
      }
      current_state = 3;
      break;
    
    
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

  data_time_arr[data_counter] = collection_time; // show what time we collected data

  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }

  // get accel rates in g's
  accel_x = imu.calcAccel(imu.ax)-accel_x_cal;
  accel_y = imu.calcAccel(imu.ay)-accel_y_cal;
  accel_z = imu.calcAccel(imu.az)-accel_z_cal;
  
  // get gyro rates in rad/s
  gyro_x = (imu.calcGyro(imu.gx)-gyro_x_cal)*deg_to_rad;
  gyro_y = (imu.calcGyro(imu.gy)-gyro_y_cal)*deg_to_rad;
  gyro_z = (imu.calcGyro(imu.gz)-gyro_z_cal)*deg_to_rad;

  // get gyro rates in rad/s
  mag_x = imu.calcMag(imu.mx);
  mag_y = imu.calcMag(imu.my);
  mag_z = imu.calcMag(imu.mz);

  altitude = altimeter.readAltitude(); // get altitude in [m]

  Serial.print(accel_x);
  Serial.print(" ");
  Serial.print(accel_y);
  Serial.print(" ");
  Serial.println(accel_z);

  // if data_counter is less than the maximum, add data to the arrays
  if (data_counter < max_data_count){
      accel_arr[data_counter][0] = accel_x;
      accel_arr[data_counter][1] = accel_y;
      accel_arr[data_counter][2] = accel_z;

      gyro_arr[data_counter][0] = gyro_x;
      gyro_arr[data_counter][1] = gyro_y;
      gyro_arr[data_counter][2] = gyro_z;

      mag_arr[data_counter][0] = mag_x;
      mag_arr[data_counter][1] = mag_y;
      mag_arr[data_counter][2] = mag_z;

      altitude_arr[data_counter] = altitude;
    }
}

void calibrate_sensors() {
  for (int i = 0; i < cal_reading_count; i++) {
    delay(10);
    
    imu.readAccel();
    imu.readGyro();
    
    accel_x_cal += imu.calcAccel(imu.ax);
    accel_y_cal += imu.calcAccel(imu.ay);
    accel_z_cal += imu.calcAccel(imu.az);

    gyro_x_cal += imu.calcGyro(imu.gx);
    gyro_y_cal += imu.calcGyro(imu.gy);
    gyro_z_cal += imu.calcGyro(imu.gz);
  }
  accel_x_cal = accel_x_cal/cal_reading_count;
  accel_y_cal = accel_y_cal/cal_reading_count;
  accel_z_cal = accel_z_cal/cal_reading_count - 1;

  gyro_x_cal = gyro_x_cal/cal_reading_count;
  gyro_y_cal = gyro_y_cal/cal_reading_count;
  gyro_z_cal = gyro_z_cal/cal_reading_count;
}

// transmitting live telemetry
void transmit_data() {
  
}

