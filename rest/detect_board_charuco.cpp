#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>

namespace {
const char* about = "Pose estimation using a ChArUco board";
const char* keys  =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{sl       |       | Square side length (in meters) }"
        "{ml       |       | Marker side length (in meters) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{c        |       | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }"
        "{r        |       | show rejected candidates too }";
}

int main(){
    int dictionary_id =10;
    double marker_size = 0.0152;
    double square_size = 0.0226;
    int num_x = 5;
    int num_y = 5;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(num_x, num_y, square_size, marker_size, dictionary);
    cv::Mat camMatrix(3, 3, CV_64FC1);
    cv::Mat distCoeffs(4, 1, CV_64FC1);
    camMatrix.at<double>(0,0)=1311.8916351174416;
    camMatrix.at<double>(0,1)=0;
    camMatrix.at<double>(0,2)=1312.5544091130773;
    camMatrix.at<double>(1,0)=0;
    camMatrix.at<double>(1,1)=1339.728842014348;
    camMatrix.at<double>(1,2)=987.1218186592131;
    camMatrix.at<double>(2,0)=0;
    camMatrix.at<double>(2,1)=0;
    camMatrix.at<double>(2,2)=1;

    distCoeffs.at<double>(0,0)=-0.29967733266122626;
    distCoeffs.at<double>(1,0)=0.06912806448497165;
    distCoeffs.at<double>(2,0)=-0.0008787283393230757;
    distCoeffs.at<double>(3,0)=-0.0009714247718763933;

    float axisLength = 0.5f * ((float)cv::min(5, 5) * (0.04));
    cv::Ptr<cv::aruco::DetectorParameters> params = new cv::aruco::DetectorParameters;
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE; 

    cv::VideoCapture inputVideo;
    inputVideo.open("record.avi");

    while(inputVideo.grab())
    {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);
        image.copyTo(imageCopy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image,dictionary,corners,ids,params);
        // std::vector<std::vector<cv::Point2f>> allCharucoCorners;
        // std::vector<std::vector<int>> allCharuIds;

        cv::Vec3d rvec, tvec;
        int interpolateCOrners = 0;

        if(ids.size()>0){
            std::vector<int>charucoIds;
            std::vector<cv::Point2f> charucoCorners; 
            cv::aruco::drawDetectedMarkers(imageCopy,corners,ids);
            cv::aruco::interpolateCornersCharuco(corners,ids,image,board,charucoCorners,charucoIds);
            if(charucoIds.size()>0){
                cv::aruco::drawDetectedCornersCharuco(imageCopy,charucoCorners,charucoIds,cv::Scalar(255,0,0));
                if(cv::aruco::estimatePoseCharucoBoard(charucoCorners,charucoIds,board,camMatrix,distCoeffs,rvec,tvec)){
                    cv::aruco::drawAxis(imageCopy,camMatrix,distCoeffs,rvec,tvec,axisLength);
                }
            }
            // if(charucoIds.size() > 0){
            //     allCharuIds.push_back(charucoIds);
            //     allCharucoCorners.push_back(charucoCorners);
            // }
            cv::imshow("out",imageCopy);
            char key = (char)cv::waitKey(0);
            if(key == 27) break;
        }
    }


    return 0;
}