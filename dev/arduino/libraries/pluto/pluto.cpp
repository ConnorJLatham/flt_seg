#include "Arduino.h"
#include "venus.h"
#include "Vector.h"
#include "pluto.h"
#include "SparkFunLSM9DS1.h"

Rocket::Rocket(float run_time, int start_state) : StateMachine(run_time, start_state) {
    accel_x_data.setStorage(accel_x_data_storage_array);
    accel_y_data.setStorage(accel_y_data_storage_array);
    accel_z_data.setStorage(accel_z_data_storage_array);
    accel_t_data.setStorage(accel_t_data_storage_array);

    gyro_x_data.setStorage(gyro_x_data_storage_array);
    gyro_y_data.setStorage(gyro_y_data_storage_array);
    gyro_z_data.setStorage(gyro_z_data_storage_array);
    gyro_t_data.setStorage(gyro_t_data_storage_array);

    alt_h_data.setStorage(alt_h_data_storage_array);
    alt_t_data.setStorage(alt_t_data_storage_array);

}

void Rocket::accel_read() {
    accel_t_data.push_back(millis());
    imu.readAccel();
    accel_x_data.push_back(imu.calcAccel(imu.ax));
    accel_y_data.push_back(imu.calcAccel(imu.ay));
    accel_z_data.push_back(imu.calcAccel(imu.az));
}