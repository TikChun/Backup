#include "librealsense2/rs.hpp"
#include "opencv2/tracking/tracker.hpp"
#include "opencv2/core/ocl.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/dnn.hpp"
#include "/home/tikchuntong/realsense/librealsense/wrappers/opencv/cv-helpers.hpp"

const size_t inWidth      = 300;
const size_t inHeight     = 300;
const float WHRatio       = inWidth / (float)inHeight;
const float inScaleFactor = 0.007843f;
const float meanVal       = 127.5;
const char* classNames[]  = {"background",
                             "aeroplane", "bicycle", "bird", "boat",
                             "bottle", "bus", "car", "cat", "chair",
                             "cow", "diningtable", "dog", "horse",
                             "motorbike", "person", "pottedplant",
                             "sheep", "sofa", "train", "tvmonitor"};



int main(){
    using namespace cv;
    using namespace cv::dnn;
    using namespace rs2;
    

    double cent_x = 0.0;
    double cent_y = 0.0;
    bool first_time = true;
    pipeline pipe;
    auto config = pipe.start();
    auto profile = config.get_stream(RS2_STREAM_COLOR)
                         .as<video_stream_profile>();
    rs2::align align_to(RS2_STREAM_COLOR);

    Size cropSize;
    if (profile.width() / (float)profile.height() > WHRatio)
    {
        cropSize = Size(static_cast<int>(profile.height() * WHRatio),
                        profile.height());
    }
    else
    {
        cropSize = Size(profile.width(),
                        static_cast<int>(profile.width() / WHRatio));
    }

    Rect crop(Point((profile.width() - cropSize.width) / 2,
                    (profile.height() - cropSize.height) / 2),
              cropSize);

    Ptr<Tracker> tracker;
    tracker = cv::TrackerCSRT::create();
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    const auto second_window_name = "Depth Image";
    namedWindow(second_window_name,WINDOW_AUTOSIZE);
    cv::Rect2d bbox(480,270,100,100);

    while (getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        // Wait for the next set of frames
        auto data = pipe.wait_for_frames();
        // Make sure the frames are spatially aligned
        data = align_to.process(data);

        auto color_frame = data.get_color_frame();
        auto depth_frame = data.get_depth_frame();

        // If we only received new depth frame, 
        // but the color did not update, continue
        static int last_frame_number = 0;
        if (color_frame.get_frame_number() == last_frame_number) continue;
        last_frame_number = static_cast<int>(color_frame.get_frame_number());

        // Convert RealSense frame to OpenCV matrix:
        auto color_mat = frame_to_mat(color_frame);
        auto depth_mat = depth_frame_to_meters(depth_frame);

        if(first_time){
            cv::rectangle(color_mat,bbox,cv::Scalar(255,0,0), 2, 1);
            tracker->init(color_mat,bbox);
            first_time = false;
            }
        else{
            bool ok = tracker->update(color_mat,bbox);
            if(ok){
                cv::rectangle(color_mat,bbox,cv::Scalar(255,0,0),2,1); 
                cv::rectangle(depth_mat,bbox,cv::Scalar(255,0,0),2,1); 
                first_time = false;
            }
            else{
                // printf("failed to track \n");
                cv::Rect2d bbox(480,270,100,100);
                tracker = cv::TrackerCSRT::create();
                // tracker->init(color_mat,bbox);
                first_time = true;
                
            }
        }


        // std::cout<<bbox<<std::endl;
        cent_x = bbox.x + bbox.width/2;
        cent_y = bbox.y + bbox.height/2;
        // printf("Distance %.2f\n",depth_frame.get_distance(cent_x,cent_y));
        double distance = 0;
        for(int x = cent_x - 1; x <= cent_x + 1; x++){
            for(int y =cent_y - 1; y <= cent_y + 1; y++){
                distance += depth_frame.get_distance(x,y);
            }
        }
        distance = distance / 9;
        std::string text = "Distance: " + std::to_string(distance);
        cv::Point2d location(bbox.x,bbox.y);
        putText(color_mat,text,location,FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255),2);
        putText(depth_mat,text,location,FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255),2);
        imshow(window_name, color_mat);
        imshow(second_window_name,depth_mat);
        
        // std::cout<<depth_mat<<std::endl;
        
        if(waitKey(1)==105){    // press I for retarget
            printf("Retarget\n");
            tracker.reset();
            bbox.x = color_mat.size().width / 2;
            bbox.y = color_mat.size().height / 2;

            // cv::Rect2d bbox(color_mat.size().width / 2,color_mat.size().height / 2,100,100);
            tracker = cv::TrackerCSRT::create();
            first_time = true;
        }
    }

    return 0;
}

//g++ realsense_stream.cpp -o realsense_stream $(pkg-config --cflags --libs opencv4) -lrealsense2