#ifndef __REALSENSEPOSE__
#define __REALSENSEPOSE__

#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>
#include<iostream>
#include<mutex>

struct float3 {                                                                                                                    
    float x, y, z;  
    float3 operator*(float t)
    {   
        return { x * t, y * t, z * t };
    } 

    float3 operator-(float t)
    {   
        return { x - t, y - t, z - t };
    }   

    void operator*=(float t)
    {   
        x = x * t;
        y = y * t;
        z = z * t;
    }   

    void operator=(float3 other)
    {   
        x = other.x;
        y = other.y;
        z = other.z;
    }   

    void add(float t1, float t2, float t3) 
    {   
        x += t1; 
        y += t2; 
        z += t3; 
    }   
};                     


class Rotation_Estimator
{
private:
    float PI = 3.14;
    float3 theta;
    std::mutex theta_mtx;
    float alpha = 0.98;
    bool first = true;
    double last_ts_gyro = 0;

public:

    void process_gyro(rs2_vector gyro_data, double ts);
    void process_accel(rs2_vector accel_data);
    float3 get_theta();    

};












#endif //_REALSENSEPOSE