#include "opencv2/tracking/tracker.hpp"
#include "opencv2/core/ocl.hpp"
#include <opencv2/highgui.hpp>

int main(){
    cv::Ptr<cv::Tracker> tracker;
    tracker = cv::TrackerCSRT::create();
    cv::VideoCapture video("record.avi");
    if(!video.isOpened()){
        std::cout<<"Failed to read video"<<std::endl;
        return 0;
    }

    cv::Mat frame;
    bool ok = video.read(frame);
    cv::Rect2d bbox(287,23,86,320);
    cv::rectangle(frame,bbox,cv::Scalar(255,0,0), 2, 1);
    cv::imshow("Tracking", frame);
    tracker->init(frame,bbox);

    while(video.read(frame)){
        double timer = (double)cv::getTickCount();
        bool ok = tracker->update(frame,bbox);
        if(ok){
            cv::rectangle(frame,bbox,cv::Scalar(255,0,0));
        }
        else printf("failed to track \n");

        cv::imshow("Tracking",frame);
        int k = cv::waitKey(1);
        if(k == 27) break;
    }



    return 0;
}