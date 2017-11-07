#include "CVInclude.h"
#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>

cv::VideoCapture cap;

int main (int argc, char** argv)
{

	//ros node stuff
	ros::init (argc, argv, "linedetector_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher threshpub = it.advertise ("thresholded", 1);
	image_transport::Publisher detectpub = it.advertise ("detected", 1);
	ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("/line", 100);
	geometry_msgs::Vector3 pixelLine;
	ros::Rate loop_rate (500);

	// cv stuff
	cv::Mat frame;

	// scaling factors
    double imScale = 1.00;
	
	//camera choice
	camera = argv [1][0] - 48;

	if (camera < 0)
	ROS_ERROR_STREAM ("Not a valid camera address");

	cap.open (camera);

	if (!cap.isOpend()) {
		std::cout << "Unable to open camera " << camera << std::endl;
		ROS_ERROR_STREAM ("Unable to open camera");
		return -1;
	}

	while (nh.ok()) {

		cap >> frame;
		if (!frame.empty()) {
			cv::Mat thresh, gray, ero, dila, opening, closing;
			cv::cvtColor (frame, gray, CV_BGR2GRAY);
			cv::adaptiveThreshold (gray, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 7, 0);
			cv::morphologyEx (thresh, closing, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2),
			cv::morphologyEx (closing, opening, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2),

			if (cv::waitKey(1) == 113)
			break;
		}

		else {
			ROS_ERROR_STREAM ("No Image found!");
			return -1;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
