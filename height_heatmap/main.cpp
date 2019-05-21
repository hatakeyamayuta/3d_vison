#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "realsense_pose.h"

void points_to_pcl(const rs2::video_frame& color, const rs2::depth_frame& depth_frame,const rs2_intrinsics& depth_intr,float3 theta){
    float point[3];
    Eigen::Vector3d r_point;
    Eigen::Vector3d r_out_point(3);
    float hmin = -1.0;
    float hmax = 0.5;
    float range = 1.0;
	float array[480][640] = {};
	int ar[480][640] = {};
    int count = 0;

    Eigen::Matrix3d rod;
    //rod = Eigen::AngleAxisd(theta.x,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(theta.y,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(theta.z,Eigen::Vector3d::UnitZ());
    rod = Eigen::AngleAxisd(-(theta.z+M_PI/2),Eigen::Vector3d::UnitX());

    cv::Mat colorr(cv::Size(color.get_width(), color.get_height()), CV_8UC3, (void *) color.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat heatmap(cv::Size(color.get_width(), color.get_height()), CV_8UC1,cv::Mat::AUTO_STEP);
       
    //std::cout << rod << std::endl;
	
    for (int i = 0; i < colorr.rows; i++) {
        for (int j = 0; j < colorr.cols; j++) {
            float pixel[2] = {float(j), float(i)};
            auto depth_point = depth_frame.get_distance(j, i);
            rs2_deproject_pixel_to_point(point, &depth_intr, pixel, depth_point);
            r_point << point[0],-point[1],point[2];
            r_point = rod*r_point;
        	if (r_point[1] > hmin && r_point[1] < hmax && r_point[2] < 1.0 && r_point[2] > 0) 
            {      
                array[i][j] = r_point[1] - hmin;
            }else{
                array[i][j] = 0;
            }
            count++;
        }
        
    }

	
    for(int i=0;i<colorr.rows;++i){
        for(int j=0;j<colorr.cols;++j){
            ar[i][j] = int(array[i][j]/(hmax-hmin)*255);
            heatmap.at<char>(i,j) = ar[i][j];
            //std::cout << ar[i][j]  << std::endl;
        }
    }
    cv::Mat dist;


	cv::applyColorMap(heatmap,dist,cv::COLORMAP_JET);
    cv::imwrite("test.jpg",dist);
    
    namedWindow("Display ColorImage", cv::WINDOW_AUTOSIZE );
    imshow("Display ColorImage", colorr);
    cv::imshow("HeatMap",dist);
	cv::waitKey(1);
}
int main(int args, char **argv)
{
    rs2::colorizer color_map;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    //cfg.enable_stream(RS2_STREAM_POSE);

    float g = 9.8;
    float ax,ay,az;
    float x,y,z;
    float PI=3.14;
    Rotation_Estimator algo;


    rs2::pipeline pipe;
    auto profile = pipe.start(cfg);

    auto depth_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto depth_intr = depth_stream.get_intrinsics();

    //set_depth_filters
    rs2::decimation_filter dec_filter;
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1);
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);
    rs2::spatial_filter spat_filter;
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    spat_filter.set_option(RS2_OPTION_HOLES_FILL, 0);
    rs2::temporal_filter temp_filter;
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);
    rs2::hole_filling_filter hf_filter;
    hf_filter.set_option(RS2_OPTION_HOLES_FILL, 1);
    float3 theta;
    while(1) {
        rs2::frameset frames = pipe.wait_for_frames();
        //align_fremes
        rs2::align align(RS2_STREAM_COLOR);
        auto align_frames = align.process(frames);
        auto color_frame = align_frames.first(RS2_STREAM_COLOR);
        auto depth_frame = align_frames.get_depth_frame();

        //depth_filters      
        depth_frame = dec_filter.process(depth_frame);
        depth_frame = depth_to_disparity.process(depth_frame);
        depth_frame = spat_filter.process(depth_frame);
        depth_frame = temp_filter.process(depth_frame);
        depth_frame = disparity_to_depth.process(depth_frame);
        depth_frame = hf_filter.process(depth_frame);
        
         if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
        {   
            rs2_vector gyro_sample = gyro_frame.get_motion_data();
            auto ts=gyro_frame.get_timestamp();
            algo.process_gyro(gyro_sample,ts);
            auto theta = algo.get_theta();
        
            //std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
             // ...
        }
        if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
        {

            rs2_vector accel_sample = accel_frame.get_motion_data();
            algo.process_accel(accel_sample);
            theta = algo.get_theta();
            ax = theta.x;
            ay = theta.y;                                                                                                               
            az = theta.z;
            //std::cout << "theta:" << theta.x*180/PI<< ", " << theta.y*180/PI << ", " << -(theta.z+M_PI/2)*180/PI<<std::endl;
            //x = atan2(ax,sqrt(ay*ay+az*az))*180/3.14;
            //y = atan2(ay,az)*180/3.14;
            //std::cout << "Accel:" << ax << ", " << ay << ", " << az ;
            //...
         }  
        
        points_to_pcl(color_frame,depth_frame,depth_intr,theta);
          

      
  }
  return 0;
}
