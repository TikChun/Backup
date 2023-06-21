#include "librealsense2/rs.hpp"
#include "/home/tikchuntong/realsense/librealsense/examples/example.hpp"
#include "dart/external/imgui/imgui.h"
#include "/home/tikchuntong/realsense/librealsense/third-party/imgui/imgui_impl_glfw.h"
#include "sstream"
#include "iostream"
#include "iomanip"
#include "Eigen/Dense"

std::string pretty_time(std::chrono::nanoseconds duration);
void draw_seek_bar(rs2::playback& playback, int* seek_pose, float2& location, float width);


int main(int argc, char * argv[]){
    window app(1280,720,"RealSense Post Processing Example");
    ImGui_ImplGlfw_Init(app,false);

    bool recorded = false;
    bool recording = false;

    texture depth_image;
    
    rs2::frameset frames;
    rs2::frame depth;

    rs2::colorizer color_map;

    auto pipe = std::make_shared<rs2::pipeline>();
    pipe->start();

    int seek_pos;

    rs2::device device;
    device = pipe->get_active_profile().get_device();

    // while(app){
    //     if(!device.as<rs2::playback>()){
    //         frames = pipe->wait_for_frames();
    //         depth = color_map.process(frames.get_depth_frame());
    //     }
    // }
    if(!device.as<rs2::playback>()){
        frames = pipe->wait_for_frames();
        depth = color_map.process(frames.get_depth_frame());
    }
    if(!device.as<rs2::recorder>()){
        pipe->stop();
        pipe = std::make_shared<rs2::pipeline>();
        rs2::config cfg;
        cfg.enable_record_to_file("a.bag");
        pipe->start(cfg);
        device = pipe ->get_active_profile().get_device();
    }
    else{
        device.as<rs2::recorder>().resume();
    }


    return 0;
}
//g++ record.cpp -o record -lrealsense2 -lGL -lglut -lGLU -lglfw