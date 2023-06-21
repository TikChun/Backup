#include "librealsense2/rs.hpp"
#include "iostream"

int main(int argc, char * argv[]){
    rs2::pipeline p;
    p.start();
    bool finished = true;
    while(true)
    {
        rs2::frameset frames = p.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();
        auto width = depth.get_width();
        auto height = depth.get_height();

        float dist_to_center = depth.get_distance(width / 2, height / 2);

        std::cout << "The camera is facing an objec" << dist_to_center << "meters away \r";    
        // finished = false; 
    }
    return 0;
}
//g++ hello_realsense.cpp -o hello_realsense -lrealsense2