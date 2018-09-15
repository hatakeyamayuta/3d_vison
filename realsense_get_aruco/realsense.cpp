//
// Created by htk on 8/31/18.
//
#include "realsense.h"
#include <vector>
// Constructor
RealSense::RealSense()
{
    // Initialize
    initialize();
}

// Destructor
RealSense::~RealSense()
{
    // Finalize
    finalize();
}

// Processing
void RealSense::run()
{
    // Main Loop
    while( true ){
        // Update Data
        update();

        // Draw Data
        draw();

        // Show Data
        show();


        // Key Check
        const int32_t key = cv::waitKey( 10 );
        if( key == 'q' ){
            break;
        }
    }
}

// Initialize
void RealSense::initialize()
{
    cv::setUseOptimized( true );

    // Initialize Sensor
    initializeSensor();
}

// Initialize Sensor
inline void RealSense::initializeSensor()
{
    // Set Device Config
    rs2::config config;
    config.enable_stream( rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8, color_fps );
    config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, color_width, color_height,rs2_format::RS2_FORMAT_Z16, color_fps );
    // Start Pipeline
    pipeline_profile = pipeline.start( config );
}

// Finalize
void RealSense::finalize()
{
    // Close Windows
    cv::destroyAllWindows();

    // Stop Pipline
    pipeline.stop();
}

// Update Data
void RealSense::update()
{
    // Update Frame
    updateFrame();

    // Update Color
    updateColor();
}

// Update Frame
inline void RealSense::updateFrame()
{
    // Update Frame
    frameset = pipeline.wait_for_frames();
}

// Update Color
inline void RealSense::updateColor()
{
    // Retrieve Color Frame
    color_frame = frameset.get_color_frame();

    // Retrive Frame Size
    color_width = color_frame.as<rs2::video_frame>().get_width();
    color_height = color_frame.as<rs2::video_frame>().get_height();
}

// Draw Data
void RealSense::draw()
{
    // Draw Color
    drawColor();
}

// Draw Color
inline void RealSense::drawColor()
{
    // Create cv::Mat form Color Frame
    color_mat = cv::Mat( color_height, color_width, CV_8UC3, const_cast<void*>( color_frame.get_data() ) );
}

// Show Data
void RealSense::show()
{
    // Show Color
    //showColor();

    showaruco();
}

// Show Color
inline void RealSense::showColor()
{
    if( color_mat.empty() ){
        return;
    }

    // Show Color Image
    cv::imshow( "Color", color_mat );
}

inline void RealSense::showaruco()
{
    cv::aruco::detectMarkers(color_mat, dictionary, marker_corners, marker_ids, parameters);
    cv::aruco::drawDetectedMarkers(color_mat,marker_corners,marker_ids);

    if(marker_corners.empty())
    {
         std::cout << "not found aruco" << "\r";
    } else {

        std::cout <<marker_ids.size() << " ID " << marker_ids.front() << std::endl;

        arucodepth();
        //std::cout << marker_corners.front().size() << "Pt"<< marker_corners.front() << std::endl;
    }
    cv::imshow("aruco",color_mat);
}

void RealSense::arucodepth()
{
    pt = marker_corners.front();
    rs2::depth_frame depth_frame = frameset.get_depth_frame();

    center.x = 0;
    center.y = 0;
    for(int i = 0; i < corners; i++ )
    {
        //std::cout << depth_frame.get_distance(pt[i].x,pt[i].y) << std::endl;
        center += pt[i];
    }
    int x = (int) center.x / 4;
    int y = (int) center.y / 4;
    std::cout << depth_frame.get_distance(x,y) <<" m"<< std::endl;
}

