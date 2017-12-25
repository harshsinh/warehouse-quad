#include <ros/ros.h>
#include <boost/thread.hpp>

#include "kalman.h"
#include "marker.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "detection");
	ros::start();

    //call ekf for height and orientation estimation
	HMDETECTION::EKF ekf;
	boost::thread height(&HMDETECTION::EKF::subscriber, &ekf);

	HMDETECTION::MARKER marker;
	boost::thread detection(&HMDETECTION::MARKER::subscriber, &marker);


	ros::MultiThreadedSpinner spinner(8);

	spinner.spin();

}
