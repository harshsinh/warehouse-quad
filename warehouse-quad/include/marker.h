/*
 * marker.h
 *
 *  Created on: Nov 28, 2017
 *      Author: krishna
 */

#ifndef MARKER_H_
#define MARKER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <warehouse_quad/markerInfo.h>

#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "zbar.h"

using namespace std;

namespace HMDETECTION{
class MARKER{
public:
	MARKER();
	void subscriber();
	void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
private:
    zbar::ImageScanner scanner_;
    warehouse_quad::markerInfo qr;
    ros::Publisher markerPub;
    vector <string> barcodes;

};
}

#endif /* MARKER_H_ */
