#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <iostream>
#include<mutex>
float PI =3.14;
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

class rotation_estimator
{
	float3 theta;
	std::mutex theta_mtx;
	float alpha = 0.98;
	bool first = true;
	double last_ts_gyro = 0;
public:
    void process_gyro(rs2_vector gyro_data, double ts)
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

    void process_accel(rs2_vector accel_data)
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
    
    float3 get_theta()
    {
        std::lock_guard<std::mutex> lock(theta_mtx);
        return theta;
    }
};



int main(){

	rs2::pipeline pipe;

	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_GYRO);
	cfg.enable_stream(RS2_STREAM_ACCEL);
	cfg.enable_stream(RS2_STREAM_POSE);

	pipe.start(cfg);

	float g = 9.8;
	float ax,ay,az;
	float x,y,z;
    rotation_estimator algo;
	while (1) // Application still alive?
	{
    	rs2::frameset frameset = pipe.wait_for_frames();
	
    	// Find and retrieve IMU and/or tracking data


   		 if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
	    {	
    	    rs2_vector gyro_sample = gyro_frame.get_motion_data();
            auto ts=gyro_frame.get_timestamp();
            algo.process_gyro(gyro_sample,ts);
            auto theta = algo.get_theta();
        
        	//std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
       		 // ...
    	}
    	if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
    	{

			rs2_vector accel_sample = accel_frame.get_motion_data();
            algo.process_accel(accel_sample);
            auto theta = algo.get_theta();
			ax = theta.x;
			ay = theta.y;
			az = theta.z;
    	    std::cout << "theta:" << theta.x*180/PI<< ", " << theta.y*180/PI << ", " << theta.z*180/PI ;
			//x = atan2(ax,sqrt(ay*ay+az*az))*180/3.14;
			//y = atan2(ay,az)*180/3.14;
    	    //std::cout << "Accel:" << ax << ", " << ay << ", " << az ;
       		//...
   		 }	
    	if (rs2::pose_frame pose_frame = frameset.first_or_default(RS2_STREAM_POSE))
	    {	
    	    rs2_pose pose_sample = pose_frame.get_pose_data();
  		    //std::cout << "Pose:" << pose_sample.translation.x << ", " << pose_sample.translation.y << ", " << pose_sample.translation.z ;
       		 //...
	    }
		std::cout<<"\r";
	}
}
