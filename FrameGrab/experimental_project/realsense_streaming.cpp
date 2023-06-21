#include "librealsense2/rs.hpp"
#include "opencv2/tracking/tracker.hpp"
#include "opencv2/core/ocl.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/dnn.hpp"
#include "/home/tikchuntong/realsense/librealsense/wrappers/opencv/cv-helpers.hpp"
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/detection/vpDetectorDataMatrixCode.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpV4l2Grabber.h>
#include "iostream"
#endif

const size_t inWidth = 300;
const size_t inHeight = 300;
const float WHRatio = inWidth / (float)inHeight;
const float inScaleFactor = 0.007843f;
const float meanVal = 127.5;
const char *classNames[] = {"background",
                            "aeroplane", "bicycle", "bird", "boat",
                            "bottle", "bus", "car", "cat", "chair",
                            "cow", "diningtable", "dog", "horse",
                            "motorbike", "person", "pottedplant",
                            "sheep", "sofa", "train", "tvmonitor"};

int main()
{
    using namespace cv;
    using namespace cv::dnn;
    using namespace rs2;
    bool qr_code = true; // true for qr_code, false for data matrix
    bool init_uncomplete = true;
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
    vpDetectorBase *detector = NULL;

    while (true)
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
        if (color_frame.get_frame_number() == last_frame_number)
            continue;
        last_frame_number = static_cast<int>(color_frame.get_frame_number());

        // Convert RealSense frame to OpenCV matrix:
        auto color_mat = frame_to_mat(color_frame);
        auto depth_mat = depth_frame_to_meters(depth_frame);

        vpImage<unsigned char> I;

        vpImageConvert::convert(color_mat, I);

        if (init_uncomplete)
        {
            // These insturctions will only be run once
            #if defined(VISP_HAVE_X11)
                        vpDisplayX d(I);
            #elif defined(VISP_HAVE_GDI)
                        vpDisplayGDI d(I, vpDisplay::SCALE_AUTO);
            #elif defined(VISP_HAVE_OPENCV)
                        vpDisplayOpenCV d(I, vpDisplay::SCALE_AUTO);
            #elif defined(VISP_HAVE_GTK)
                        vpDisplayGTK d(I, vpDisplay::SCALE_AUTO);
            #elif defined(VISP_HAVE_D3D9)
                        vpDisplayD3D d(I, vpDisplay::SCALE_AUTO);
            #else
                        std::cout << "No image viewer is available..." << std::endl;
            #endif

            // Use Zbar and libdmtx for QRcode detection
            if (qr_code)
            {
                detector = new vpDetectorQRCode;
            }
            else
            {
                detector = new vpDetectorDataMatrixCode;
            }

            init_uncomplete = false;
            vpDisplay::display(I);
            vpDisplay::flush(I);

            // usleep(300000);
            std::cout << "A click to quit..." << std::endl;
            vpDisplay::getClick(I);

            continue;
        }

        #if defined(VISP_HAVE_X11)
                vpDisplayX d(I);
        #elif defined(VISP_HAVE_GDI)
                vpDisplayGDI d(I, vpDisplay::SCALE_AUTO);
        #elif defined(VISP_HAVE_OPENCV)
                vpDisplayOpenCV d(I, vpDisplay::SCALE_AUTO);
        #elif defined(VISP_HAVE_GTK)
                vpDisplayGTK d(I, vpDisplay::SCALE_AUTO);
        #elif defined(VISP_HAVE_D3D9)
                vpDisplayD3D d(I, vpDisplay::SCALE_AUTO);
        #else
                std::cout << "No image viewer is available..." << std::endl;
        #endif

        vpDisplay::display(I);

        bool status = detector->detect(I);
        std::ostringstream legend;
        legend << detector->getNbObjects() << "bar code detected";
        if (status)
            vpDisplay::displayText(I, 10, 10, legend.str(), vpColor::red);
        if (status)
        {
            for (size_t i = 0; i < detector->getNbObjects(); i++)
            {
                std::vector<vpImagePoint> p = detector->getPolygon(i);
                vpRect bbox = detector->getBBox(i);
                vpDisplay::displayRectangle(I, bbox, vpColor::green,false,5);
                
                
                for (size_t j = 0; j < p.size(); j++)
                {
                    vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
                    std::ostringstream number;
                    number << j;
                    vpDisplay::displayText(I, p[j] + vpImagePoint(10, 0), number.str(), vpColor::blue);
                }
            }
        }
        vpDisplay::flush(I);

        // usleep(300000);
        std::cout << "A click to continue..." << std::endl;
        vpDisplay::getClick(I);
    }
    delete detector;

    return 0;
}
// g++ -g `/home/tikchuntong/visp-ws/visp-build/bin/visp-config --cflags` -o realsense_streaming realsense_streaming.cpp `/home/tikchuntong/visp-ws/visp-build/bin/visp-config --libs` $(pkg-config --cflags --libs opencv4) -lrealsense2