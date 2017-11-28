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

#include "zbar.h"

namespace HMDETECTION{
class MARKER{
public:
	MARKER(ros::NodeHandle nh);
	void subscriber();
	void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
private:
	ros::NodeHandle nh_;
    zbar::ImageScanner scanner_;

};
}

#endif /* MARKER_H_ */
