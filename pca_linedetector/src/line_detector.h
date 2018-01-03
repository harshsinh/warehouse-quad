/***********************************************************
 * line_detector.h
 * Created On : 26th Dec 2017
 *      Author: Harsh Sinha (harshsin@iitk.ac.in)
 *
 * Line detector
 **********************************************************/

#ifndef __LINE_DETECTOR_H__
#define __LINE_DETECTOR_H__

#include "CVInclude.h"
#include "pca.h"
#include "line.h"
#include <vector>
#include <ros.h>
#include <ros/console.h>

namespace warehouse_pcadetector {
class LineDetector {

    public:
        LineDetector();

    public:
        Line::Line X, Y, Y_buff;
        pca::pca P;
        cv::Mat image_capture (int camera);
        std::vector<cv::Vec2i> extractPoints (cv::Mat &frame);
        void publish_images (void);

    public:
        cv::Mat frame;
        cv::VideoCapture cap;
        bool marker_detected;
        bool turn;

    private:
        int _camera;

    private:
        static const int _size = 256;
        static const bool _zed = 0;
        static const double _to_deg = 180/3.14159;
        static const cv::Scalar _yellow_low = cv::Scalar (20, 80, 155);
        static const cv::Scalar _yellow_high= cv::Scalar (40, 255, 255);

};
}
#endif
