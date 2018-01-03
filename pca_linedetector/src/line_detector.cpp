#include "line_detector.h"

namespace warehouse_pcadetector {

LineDetector::LineDetector():
    marker_detected (false),
    turn (false)
{}

cv::Mat LineDetector::image_capture (int cap)
{
    if (camera >= 0 && camera < 10)
    {
        std::cout << camera << std::endl;
        cap.open (camera);

        if (!cap.isOpened())
        {
            std::cout << "Unable to open camera" << std::end;
            ROS_ERROR_STREAM ("Unable to open camera %d", camera);
            return -1;
        }
    }
}
}