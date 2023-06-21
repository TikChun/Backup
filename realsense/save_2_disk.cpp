#include "librealsense2/rs.hpp"
#include "iostream"
#include "fstream"
#include "sstream"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "/home/tikchuntong/realsense/librealsense/third-party/stb_image_write.h"
#include "dirent.h"


void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
    std::ofstream csv;

    csv.open(filename);

    //    std::cout << "Writing metadata to " << filename << endl;
    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

    // Record all the available metadata attributes
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }

    csv.close();
}



int main(int argc, char * argv[]){
    rs2::colorizer color_map;
    rs2::pipeline pipe;
    pipe.start();
    int max = 0;

    std::string path = "/home/tikchuntong/ur_move_ws/my_code/RealSense/image";
    DIR *dir;
    struct dirent *ent;
    if((dir = opendir("/home/tikchuntong/ur_move_ws/my_code/RealSense/image")) != NULL){
        while((ent = readdir(dir))!=NULL){
            printf("%s\n", ent->d_name);
            // std::string fullpath = path + "/" +ent->d_name;
            std::string filename = ent->d_name;
            // printf("%s\n", fullpath.c_str());
            if(filename.length() > 4 && filename.substr(filename.length() - 4) == ".png"){
                std::string indexStr = filename.substr(0,filename.length()-4);
                std::istringstream iss(indexStr);
                int index;
                if(!(iss>>index)){
                    continue;
                }
                if(index > max){
                    max = index;
                }
            }
        }
        closedir(dir);
    }

    std::cout << "The maximum index is "<< max << std::endl;

    max += 1;


    for(int i = 0; i < 30; ++i) pipe.wait_for_frames();

    for(auto&& frame : pipe.wait_for_frames()){
        if(auto vf = frame.as<rs2::video_frame>()){
            auto stream = frame.get_profile().stream_type();
            if(vf.is<rs2::depth_frame>()) vf = color_map.process(frame);
            std::stringstream png_file;
            auto current = std::chrono::system_clock::now();
            std::time_t end_time = std::chrono::system_clock::to_time_t(current);

            // png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << std::ctime(&end_time) << ".png";
            // stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
            //                vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
            // std::cout<<"Saved"<<png_file.str()<<std::endl;
            

            png_file << std::to_string(max) << ".png";
            stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                           vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
            std::cout<<"Saved"<<png_file.str()<<std::endl;



            // std::stringstream csv_file;
            // csv_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << "-metadata.csv";
            // metadata_to_csv(vf,csv_file.str());
        }
    }

    return 0;
}