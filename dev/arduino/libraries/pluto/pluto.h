#ifndef pluto_h
#define pluto_h

#include "Arduino.h"
#include "venus.h"
#include "Vector.h"
#include "SparkFunLSM9DS1.h"

const int MAX_DATA = 20;

class Rocket: public StateMachine {
    public:
        float weight = 50.0;
        LSM9DS1 imu;
        Rocket(float run_time, int start_state);

        float accel_x_data_storage_array[MAX_DATA];
        float accel_y_data_storage_array[MAX_DATA];
        float accel_z_data_storage_array[MAX_DATA];
        float accel_t_data_storage_array[MAX_DATA];
        Vector<float> accel_x_data;
        Vector<float> accel_y_data;
        Vector<float> accel_z_data;
        Vector<float> accel_t_data;

        float gyro_x_data_storage_array[MAX_DATA];
        float gyro_y_data_storage_array[MAX_DATA];
        float gyro_z_data_storage_array[MAX_DATA];
        float gyro_t_data_storage_array[MAX_DATA];
        Vector<float> gyro_x_data;
        Vector<float> gyro_y_data;
        Vector<float> gyro_z_data;
        Vector<float> gyro_t_data;

        float alt_h_data_storage_array[MAX_DATA];
        float alt_t_data_storage_array[MAX_DATA];
        Vector<float> alt_h_data;
        Vector<float> alt_t_data;



};

#endif