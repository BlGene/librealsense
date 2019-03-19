// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <limits>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <experimental/filesystem>

// Includes for time display
#include <unistd.h>
#include <imgui.h>
#include "imgui_impl_glfw.h"
#include "opencv2/opencv.hpp"

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
namespace fs = std::experimental::filesystem;

// Helper function for dispaying time conveniently
std::string pretty_time(std::chrono::nanoseconds duration);
// Helper function for rendering a seek bar
void draw_seek_bar(rs2::playback& playback, int* seek_pos, float2& location, float width);

std::string record_dir;
std::atomic<bool> keep_recording(true);

std::vector< std::pair<int, cv::Mat> > bufferColor; // Buffered image data
std::vector< std::pair<int, cv::Mat> > bufferDepth; // Buffered depth and ir data

std::mutex lock_bufferColor;
std::mutex lock_bufferDepth;

void query_stop_recording() {
    std::cout << "Press Enter to stop recording." << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    std::cout << "Stopping recording." << std::endl;
    keep_recording = false;
}

void write_color_func() {
    while(true)
    {
        if(bufferColor.size())
        {
            std::unique_lock<std::mutex> lock(lock_bufferColor);
            std::pair<int, cv::Mat> color_pair(bufferColor.back());
            bufferColor.pop_back();
            lock.unlock();

            std::ostringstream css;
            css << record_dir << "/color_";
            css << std::setfill('0') << std::setw(8) << color_pair.first << ".png";
            auto color_filename = css.str();
            cv::imwrite(color_filename.c_str(), color_pair.second);
        }
        else if(!keep_recording)
        {
            break;
        }
        else
        {
            usleep(100);
        };
    }
}

void write_depth_func() {
    while(true)
    {
        if(bufferDepth.size())
        {
            std::unique_lock<std::mutex> lock(lock_bufferDepth);
            std::pair<int, cv::Mat> depth_pair(bufferDepth.back());
            bufferDepth.pop_back();
            lock.unlock();

            std::ostringstream css;
            css << record_dir << "/depth_";
            css << std::setfill('0') << std::setw(8) << depth_pair.first << ".png";
            auto depth_filename = css.str();
            cv::imwrite(depth_filename.c_str(), depth_pair.second);
        }
        else if(!keep_recording)
        {
            break;
        }
        else
        {
            usleep(100);
        };
    }
}


int main(int argc, char * argv[]) try
{

    // Create booleans to control GUI (recorded - allow play button, recording - show 'recording to file' text)
    bool init_record = true;  //record at initalization
    bool recorded = false;
    bool recording = false;
    uint frame_number = 0;


    // Declare a texture for the depth image on the GPU
    texture depth_image;

    // Declare frameset and frames which will hold the data from the camera
    rs2::frameset frames;
    rs2::frame depth;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    std::thread input_thread(&query_stop_recording);
    std::thread write_color_thread(&write_color_func);
    std::thread write_depth_thread(&write_depth_func);

    // Create a shared pointer to a pipeline
    auto pipe = std::make_shared<rs2::pipeline>();

    // Start streaming with default configuration
    pipe->start();
    // Initialize a shared pointer to a device with the current device on the pipeline
    rs2::device device = pipe->get_active_profile().get_device();

    std::cout << "Starting to record!" << std::endl;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "record_%Y%m%d%H%M%S");
    record_dir = oss.str();

    fs::create_directory(record_dir);

    usleep(10);


    pipe->stop(); // Stop the pipeline with the default configuration


    rs2::config cfg; // Declare a new configuration

    int FPS = 6;
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8 , FPS);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, FPS);
    // enable this to ros-bag
    //cfg.enable_record_to_file(record_filename);
    rs2::pipeline_profile selection = pipe->start(cfg); //File will be opened at this point

    //auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH);
    //auto color_stream = selection.get_stream(RS2_STREAM_COLOR);
    //rs2_extrinsics e = depth_stream.get_extrinsics_to(color_stream);
    //std::cout << "extrinsics: " <<e << endl;

    auto color_stream = selection.get_stream(RS2_STREAM_COLOR)
                                 .as<rs2::video_stream_profile>();
    auto color_resolution = std::make_pair(color_stream.width(), color_stream.height());

    auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH)
                                 .as<rs2::video_stream_profile>();
    auto depth_resolution = std::make_pair(depth_stream.width(), depth_stream.height());


    std::cout << "color res: " << color_resolution.first << " " << color_resolution.second << std::endl;
    std::cout << "depth res: " << depth_resolution.first << " " << depth_resolution.second << std::endl;

    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    auto scale =  depth_sensor.get_depth_scale();

    std::cout << "scale: " << scale << std::endl;
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        auto emitter = depth_sensor.get_option(RS2_OPTION_EMITTER_ENABLED);
        std::cout << "emitter: "<< emitter << std::endl;
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto laser_power = depth_sensor.get_option(RS2_OPTION_LASER_POWER);
        std::cout << "laser_power: "<< laser_power << std::endl;
    }

    device = pipe->get_active_profile().get_device();

    rs2::frameset frames; // Realsense frames

    while(keep_recording){


        frames = pipe->wait_for_frames();

        std::unique_lock<std::mutex> lock(lock_bufferColor);
        std::unique_lock<std::mutex> lock(lock_bufferDepth);


        // When frames are available get the data
        rs2::frame color_frame = m_frames.get_color_frame();
        rs2::frame depth_frame = m_frames.get_depth_frame();

        cv::Mat color(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        cv::Mat color_clone(color.clone());
        cv::Mat depth_clone(depth.clone());

        bufferColor.emplace_back(std::pair<int, cv::Mat>(frame_number, std::move(color_clone)));
        bufferDepth.emplace_back(std::pair<int, cv::Mat>(frame_number, std::move(depth_clone)));



        frame_number += 1;
        }

    }
    std::cout << "exiting." << std::endl;
    input_thread.join();
    write_color_thread.join();
    write_depth_thread.join();
    std::cout << "exiting done." << std::endl;
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cout << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



