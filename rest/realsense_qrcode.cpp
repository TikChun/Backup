#include "librealsense2/rs.hpp"
#include "opencv2/tracking/tracker.hpp"
#include "opencv2/core/ocl.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/dnn.hpp"
#include "/home/tikchuntong/realsense/librealsense/wrappers/opencv/cv-helpers.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "unistd.h"

using namespace cv;
using namespace cv::dnn;
using namespace rs2;

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
/*

    Need RealSense, OpenCV, no need to install ROS

    The main perpose is to first locate the position of the QRcode, then use a tracker to keep track of it. But as time goes by, the tracker will
    be shifting to other places, in which you can press "i" in the keyboard to redetect the QRcode, and retrack it.

    In the future, we can replace the QR code detection with some other detection.
    If we replace it with some pose-detectable code. Combine with the distance data. We can calculate
    the movement needed for the robot arm to readch a certain position relative to the code.

    Known issue: at the start of the programe, if no QR code can be found within a period of time, an error will appeared.
    
*/

cv::Rect2d display(Mat &im, Mat &bbox, const char* windowsname){
    int n = bbox.rows;
    for(int i = 0; i < n; i++){
        line(im, Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), Scalar(255,0,0), 3);
        // std::cout << Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)) << "---" << Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)) << std::endl;
    }
    int x = bbox.at<float>(1,0);
    int y = bbox.at<float>(2,1);
    int width = bbox.at<float>(1,0) - bbox.at<float>(0,0);
    int height = bbox.at<float>(2,1) - bbox.at<float>(1,1);
    cv::Rect2d bbox_3(x,y,width,height);
    printf("x: %d, y: %d, height: %d, width: %d\n",x,y,height, width);
    imshow(windowsname,im);
    return bbox_3;
}

cv::Rect2d qr_find_locate(Mat &color_mat, bool &qr_found, std::string &text){
    QRCodeDetector qrDecoder = QRCodeDetector();
    Mat bbox;
    if(!qrDecoder.detect(color_mat,bbox)){
        sleep(1);
        cv::Rect2d bbox_2(10,10,10,10);
        qr_found = false;
        return bbox_2;
    }
    // text = qrDecoder.decode(color_mat,bbox);
    // for(int i = 0; i < bbox.rows; i++){
    //     line(im, Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), Scalar(255,0,0), 3);
    //     std::cout << Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)) << "---" << Point2i(bbox.at<float>((i+1) % bbox.rows,0), bbox.at<float>((i+1) % bbox.rows,1)) << std::endl;
    // }
    int x = bbox.at<float>(0,0);
    int y = bbox.at<float>(0,1);
    int width = bbox.at<float>(1,0) - bbox.at<float>(0,0);
    int height = bbox.at<float>(2,1) - bbox.at<float>(1,1);
    cv::Rect2d bbox_2(x,y,width,height);
    qr_found = true;
    // printf("x: %d, y: %d, height: %d, width: %d\n",x,y,height, width);
    return bbox_2;
}





int main(){
    
    double cent_x = 0.0;
    double cent_y = 0.0;
    bool first_time = true;
    bool qr_found = false;
    pipeline pipe;

    // fire up the camera
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
    // finish the SOP of starting a camera

    // Start creating the tracker
    Ptr<Tracker> tracker;
    tracker = cv::TrackerCSRT::create();    // Choose to use CSRT, as it's the most accuracte one
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    const auto second_window_name = "Depth Image";
    namedWindow(second_window_name,WINDOW_AUTOSIZE);
    // Prepare two windows for future use

    int bbox_width = 50;
    int bbox_height = 50;
    cv::Rect2d bbox(10,10,bbox_width,bbox_height);  // Just making a randowm bbox, the dimension does not matter
    QRCodeDetector qrDecoder = QRCodeDetector();    // Get a detector/decoder for the QRcode
    std::string text;

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
        
        if(!qr_found){
            bbox = qr_find_locate(color_mat,qr_found,text);
            imshow(window_name,color_mat);
            continue;
        }

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
                // cv::Rect2d bbox(480,270,100,100);
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

        // std::cout<<"x: "<<bbox.x<<" y: "<< bbox.y <<std::endl;
        
        // std::cout<<depth_mat<<std::endl;
        
        if(waitKey(1)==105){    // press I for retarget
            printf("Retarget\n");
            qr_found = false;
            tracker.reset();
            // cv::Rect2d bbox(color_mat.size().width / 2,color_mat.size().height / 2,100,100);
            tracker = cv::TrackerCSRT::create();
            first_time = true;
        }
        if(waitKey(1)==106){    //press J
            printf("Track anything in the middle\n");
            qr_found = true;
            tracker.reset();
            cv::Rect2d bbox(color_mat.size().width / 2,color_mat.size().height / 2,100,100);
            tracker = cv::TrackerCSRT::create();
            first_time = true;
        }
        // if(waitKey(1)!=0){
        //     printf("%d\n",waitKey(1));
        // }
    }

    return 0;
}

//g++ realsense_qrcode.cpp -o realsense_qrcode $(pkg-config --cflags --libs opencv4) -lrealsense2