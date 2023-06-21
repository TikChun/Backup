#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>
#include "filesystem"
#include "iostream"
#include "string"
#include "dirent.h"
int main(){
    int dictionary_id =10;
    double square_size = 0.0226;
    double marker_size = 0.0152;
    int num_x = 5;
    int num_y = 5;
    double aspectRatio = 1;
    
    // cv::VideoCapture inputVideo;
    // inputVideo.open("record.avi");
    cv::Ptr<cv::aruco::DetectorParameters> params = new cv::aruco::DetectorParameters;
    float axisLength = 0.5f * ((float)cv::min(5, 5) * (0.04));
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE; 

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(num_x, num_y, square_size, marker_size, dictionary);
    params->cornerRefinementMethod = 0;
    cv::namedWindow("out",cv::WINDOW_KEEPRATIO);

    std::vector < std::vector<int> > allIds;
    std::vector<std::vector<std::vector<cv::Point2f>>> allCorners;
    std::vector<cv::Mat> allImgs; 
    cv::Size imgSize;




    std::string path = "/home/tikchuntong/ur_move_ws/my_code/RealSense/image";
    DIR *dir;
    struct dirent *ent;
    if((dir = opendir("/home/tikchuntong/ur_move_ws/my_code/RealSense/image")) != NULL){
        while((ent = readdir(dir))!=NULL){
            std::string filename = ent->d_name;
            if((filename.length()<4)||(filename.substr(filename.length() - 4) != ".png")){
                continue;
            }
            // printf("%s\n", ent->d_name);
            std::string fullpath = path + "/" +ent->d_name;
            printf("%s\n", fullpath.c_str());
            cv::Mat image, imageCopy;
            image = cv::imread(fullpath);
            if(image.empty()){
                printf("failed\n");
                continue;
            }

            image.copyTo(imageCopy);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(image, dictionary, corners, ids, params);
            // if at least one marker detected

            cv::Vec3d rvec, tvec;
            int interpolateCOrners = 0;

            if (ids.size() > 0) {
                cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::aruco::interpolateCornersCharuco(corners, ids, image, board, charucoCorners, charucoIds);
                // if at least one charuco corner detected
                if(charucoIds.size() > 0)
                    cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
            }
            cv::imshow("out", imageCopy);
            char key = (char) cv::waitKey(0);
            allCorners.push_back(corners);
            allIds.push_back(ids);
            allImgs.push_back(image);
            imgSize = image.size();
            if (key == 27) break;
        }
        if(allIds.size() < 1){
            std::cout<<"not enough ids"<<std::endl;
            closedir(dir);
            return 0;
        }
        cv::Mat cameraMatrix, distCoeffs;
        std::vector<cv::Mat>rvecs,tvecs;
        double repError;

        // cameraMatrix = cv::Mat::eye(3,3,CV_64F);
        // cameraMatrix.at<double>(0,0) = aspectRatio;

        std::vector<std::vector<cv::Point2f>> allCornersConcatenated;
        std::vector<int> allIdsConcatenated;
        std::vector<int> markerCounterPerFrame;
        markerCounterPerFrame.reserve(allCorners.size());
        for(unsigned int i = 0; i < allCorners.size(); i++){
            markerCounterPerFrame.push_back((int) allCorners[i].size());
            for(unsigned j = 0; j < allCorners[i].size(); j++){
                allCornersConcatenated.push_back(allCorners[i][j]);
                allIdsConcatenated.push_back(allIds[i][j]);
            }
        }

        int nFrames = (int)allCorners.size();
        std::vector<cv::Mat> allCharucoCorners;
        std::vector<cv::Mat> allCharucoIds;
        std::vector<cv::Mat> filteredImages;
        allCharucoCorners.reserve(nFrames);
        allCharucoIds.reserve(nFrames);

        for(int i = 0; i < nFrames; i++){
            cv::Mat currentCharucoCorners, currentCharucoIds;
            cv::aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], board,
                                         currentCharucoCorners, currentCharucoIds, cameraMatrix,
                                         distCoeffs);
            allCharucoCorners.push_back(currentCharucoCorners);
            allCharucoIds.push_back(currentCharucoIds);
            filteredImages.push_back(allImgs[i]);               
        }
        if(allCharucoCorners.size() < 4){
            printf("Not enough coners\n");
            closedir(dir);
            return 0;
        }

        repError = cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, board, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs, 0);
        printf("\n,CameraMatrix:\n");
        std::cout<<cameraMatrix<<std::endl;
        printf("\nDistCoeffs:\n");
        std::cout<<distCoeffs<<std::endl;
        printf("\n");
        printf("repError: %.4f \n",repError);



        
        closedir(dir);
        return 0;
    }

    
    // while (inputVideo.grab()) {
    //     cv::Mat image, imageCopy;
    //     inputVideo.retrieve(image);
    //     image.copyTo(imageCopy);
    //     std::vector<int> ids;
    //     std::vector<std::vector<cv::Point2f>> corners;
    //     cv::aruco::detectMarkers(image, dictionary, corners, ids, params);
    //     // if at least one marker detected
    //     if (ids.size() > 0) {
    //         cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
    //         std::vector<cv::Point2f> charucoCorners;
    //         std::vector<int> charucoIds;
    //         cv::aruco::interpolateCornersCharuco(corners, ids, image, board, charucoCorners, charucoIds);
    //         // if at least one charuco corner detected
    //         if(charucoIds.size() > 0)
    //             cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
    //     }
    //     cv::imshow("out", imageCopy);
    //     char key = (char) cv::waitKey(0);
    //     if (key == 27)
    //         break;
    // }
    return 0;
}
//g++ create_marker.cpp -o create_marker $(pkg-config --cflags --libs opencv4)