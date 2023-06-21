#include "opencv2/opencv.hpp"
#include "fstream"
#define INPUT_WIDTH 640.0
#define INPUT_HEIGHT 640.0


cv::Mat format_yolov5(const cv::Mat &source){
    
    // Create a big square, and put the image inisde the square
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col,row);
    cv::Mat resized = cv::Mat::zeros(_max,_max,CV_8UC3);
    source.copyTo(resized(cv::Rect(0,0,col,row)));

    // Resize to 640x640, normalized, swap red and blue (RGB -> BGR)
    cv::Mat result;
    cv::dnn::blobFromImage(source,result,1./255., cv::Size(INPUT_WIDTH,INPUT_HEIGHT),cv::Scalar(),true,false);
    return result;
}




int main(){
    auto net = cv::dnn::readNet("yolov5s.onnx");   
    return 0;
}