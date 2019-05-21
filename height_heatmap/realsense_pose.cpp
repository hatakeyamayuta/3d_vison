#include "realsense_pose.h"
void Rotation_Estimator::process_gyro(rs2_vector gyro_data,double ts)
{
    if (first) // On the first iteration, use only data from accelerometer to set the camera's initial position
    {
        last_ts_gyro = ts;
        return;
    }
    float3 gyro_angle;

    gyro_angle.x = gyro_data.x; // Pitch
    gyro_angle.y = gyro_data.y; // Yaw
    gyro_angle.z = gyro_data.z; // Roll

    double dt_gyro = (ts - last_ts_gyro) / 1000.0;
    last_ts_gyro = ts;

    gyro_angle = gyro_angle * dt_gyro;

    std::lock_guard<std::mutex> lock(theta_mtx);
    theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);

}

void Rotation_Estimator::process_accel(rs2_vector accel_data)
    {
        float3 accel_angle;

        accel_angle.z = atan2(accel_data.y, accel_data.z);
        accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

        std::lock_guard<std::mutex> lock(theta_mtx);
        if (first)
        {
            first = false;
            theta = accel_angle;
            theta.y = PI;
        }
        else
        {
            theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
            theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
        }
    }
    
float3 Rotation_Estimator::get_theta()
    {
        std::lock_guard<std::mutex> lock(theta_mtx);
        return theta;
    }
