#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include<iostream>
#include<fstream>
#include<string>

float pixel[2];
void mouse_callback(int event, int x, int y,int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        std::cout << "mouse X=" << x << " Y= "<< y << std::endl;
        pixel[0] = x;
        pixel[1] = y;
    }
}

void get_point(const rs2::video_frame& color, const rs2::depth_frame& depth_frame,rs2_intrinsics depth_intr){

    cv::Mat colorr(cv::Size(color.get_width(), color.get_height()), CV_8UC3, (void *) color.get_data(), cv::Mat::AUTO_STEP);
    // OpenCV Mat for showing the rgb color image, just as part of processing
    namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    imshow("Display Image", colorr);
    cv::setMouseCallback("Display Image",mouse_callback);
}

int main(int argc, char** argv)
{
    float deg = 0;
	float rad = 0;
    int number = 0;
	int h = 720;
	int w = 1280;
	rs2::colorizer color_map;
	
	std::string output_name;
    std::string img_name;
  	rs2::config cfg;
  	cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, 30);
  	cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, 30);

  	rs2::pipeline pipe;
  	auto profile = pipe.start(cfg);

  	auto depth_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  	auto depth_intr = depth_stream.get_intrinsics();

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

  	rs2::frameset frames;
  	rs2::points points;
 	rs2::pointcloud pc; 

    while(1) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::align align(RS2_STREAM_COLOR);
        auto align_frames = align.process(frames);
		
        auto color_frame = align_frames.first(RS2_STREAM_COLOR);

        auto depth_frame = align_frames.get_depth_frame();

        depth_frame = dec_filter.process(depth_frame);
        depth_frame = depth_to_disparity.process(depth_frame);
        depth_frame = spat_filter.process(depth_frame);
        depth_frame = temp_filter.process(depth_frame);
        depth_frame = disparity_to_depth.process(depth_frame);
        depth_frame = hf_filter.process(depth_frame);

        get_point(color_frame, depth_frame, depth_intr);

        float point[3];

        auto key = cv::waitKey(1);
        if (key == 'q') break;

        if (key == 's')
        {
			
			
			output_name = "test_"+std::to_string(number);
            float depth_point = depth_frame.get_distance(pixel[0], pixel[1]);
    		cv::Mat colorr(cv::Size(w, h), CV_8UC3, (void *) color_frame.get_data(), cv::Mat::AUTO_STEP);
            std::cout << "deg =" << deg << std::endl;
			rad = deg/360*3.14;
            deg -=15;
            rs2_deproject_pixel_to_point(point, &depth_intr, pixel, depth_point);
			cv::imwrite(output_name+".png", colorr);
            number = number + 1;
			std::ofstream outputfile;
			outputfile.open(output_name +"_.txt",std::ios::out);
			outputfile << rad << " " << point[0]*50 << " " << point[1]*50 << " " << point[2]*50;
			outputfile.close();
			std::cout << "x=" << point[0] << " y=" << point[1] << " z=" << point[2] << std::endl;
        }
    }
  return 0;

}
