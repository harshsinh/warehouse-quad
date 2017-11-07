#include "CVInclude.h"
#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <geometry_msgs/Vector3.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

// cv stuff
cv::Mat frame;
cv::VideoCapture cap;

void imcallback (const sensor_msgs::ImageConstPtr& msg)
{
	frame = cv_bridge::toCvShare (msg) -> image;
	return;
}

int main (int argc, char** argv)
{

	//ros node stuff
	ros::init (argc, argv, "linedetector_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher threshpub = it.advertise ("thresholded", 1);
	image_transport::Publisher detectpub = it.advertise ("detected", 1);
	ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("/line", 100);
	image_transport::Subscriber sub = it.subscribe ("/usb_cam/image_raw", 1000, imcallback);
	geometry_msgs::Vector3 pixelLine;
	ros::Rate loop_rate (50);

	// scaling factors
    double imScale = 1.00;
	
	//camera choice
	int camera = argv [1][0] - 48;

	
	if (camera >= 0 && camera < 10) {
		std::cout << camera << std::endl;
		cap.open (camera);
		if (!cap.isOpened()) {
			std::cout << "Unable to open camera " << camera << std::endl;
			ROS_ERROR_STREAM ("Unable to open camera");
			return -1;
		}
	}

	while (nh.ok()) {

		// if (cap.isOpened())
		// 	cap >> frame;

		if (!frame.empty()) {
			cv::Mat  thresh, gray, ero, dila, blurred, opening, closing, result;
			cv::Mat grad_x, grad_y;
			std::vector<cv::Vec2f> lines;

			
			cv::cvtColor (frame, gray, CV_BGR2GRAY);
			cv::GaussianBlur (gray, blurred,  cv::Size(11, 11), 0, 0);
			// cv::adaptiveThreshold (blurred, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 7, 0);
			cv::threshold (blurred, thresh, 127, 255, CV_THRESH_BINARY);
			cv::morphologyEx (thresh, closing, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2),  cv::Point(-1, -1)));
			cv::morphologyEx (closing, opening, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2),  cv::Point(-1, -1)));
			// cv::Canny (thresh, result, 50, 200, 3);
			cv::Sobel( thresh, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
			cv::convertScaleAbs( grad_x, grad_x );
			cv::Sobel( thresh, grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
			cv::convertScaleAbs( grad_y, grad_y );
			cv::addWeighted( grad_x, 0.5, grad_y, 0.5, 0, result);
			
			cv::imshow("image", thresh);
			cv::imshow("openig", result);

			if (cv::waitKey(1) == 113)
			break;
		}

		// else {
		// 	ROS_ERROR_STREAM ("No Image found!");
		// 	return -1;
		// }

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
