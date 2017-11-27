/*****************************************************************************
 * 
 * Publishers : /lines -- geometry_msgs/Vector3 : x - slope, y - intercept
 * Publishers : /detected -- sensor_mags/Image  : detected Image with line
 * Subscriber : /usb_cam/image_raw -- Input Image.
 * Origin : (0, 0) at image center
 * Slope with respect to y axis
 * 
 ****************************************************************************/
#include "CVInclude.h"
#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <geometry_msgs/Vector3.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

// Frame and Camera
cv::Mat frame;
cv::VideoCapture cap;

// frame from callback
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
	ros::Rate loop_rate (50);
	geometry_msgs::Vector3 pixelLine;
	image_transport::ImageTransport it(nh);
	
	// Publish the final line detected image and line
	image_transport::Publisher threshpub = it.advertise ("thresholded", 1);
	ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("/line", 100);
	image_transport::Subscriber sub = it.subscribe ("/usb_cam/image_raw", 1000, imcallback);
	
	//camera choice
	int camera = argv [1][0] - 48;

	// Camera has to specified in digit, else go for subcription
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

		if (cap.isOpened())
			cap >> frame;

		if (!frame.empty()) {

			cv::Mat  thresh, gray, blurred, opening, closing, result, temp, final_image;

			std::vector<cv::Vec4i> lines;
			std::vector<cv::Vec2f> mc;

			cv::resize (frame, frame, cv::Size(256, 256));
			cv::cvtColor (frame, gray, CV_BGR2GRAY);
			cv::GaussianBlur (gray, blurred,  cv::Size(11, 11), 0, 0);
			cv::threshold (blurred, thresh, 127, 255, CV_THRESH_BINARY);
			cv::morphologyEx (thresh, closing, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2),  cv::Point(-1, -1)));
			cv::morphologyEx (closing, opening, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2),  cv::Point(-1, -1)));
			cv::Canny (thresh, temp, 50, 150, 3);

			result = cv::Scalar::all(0);
			opening.copyTo(result, temp);

			cv::HoughLinesP (result, lines, 1, CV_PI/180, 5);

			std::cout << lines.size() << std::endl;			
			std::cout << "******************" << std::endl;

			float temp_ = 0.0, slope = 0.0, interc = 0.0;
			int count = 0;

			for (std::vector<cv::Vec4i>::iterator i = lines.begin(); i != lines.end(); ++i) {

				cv::Vec4i l = *i;
				cv::line (frame, cv::Point (l[0], l[1]), cv::Point (l[2], l[3]), cv::Scalar(255, 0, 0), 3, CV_AA);
				temp_ = (l[3] - l[1]) ? (l[3] - l[1]) : 1;
				slope += (l[2] - l[0])/temp_;
				interc += l[1]*l[2] - l[3]*l[0];
				count += 1;

			}

			std::cout << "m : " << slope/count << " c : " << (interc/count) - 128 << std::endl;
			pixelLine.x = slope/count;
			pixelLine.y = (interc/count) - 128;
			std::cout << "******************" << std::endl;

			cv::imshow("image", thresh);
			cv::imshow("openig", result);
			cv::imshow("final image", frame);
			pub.publish(pixelLine);

			if (cv::waitKey(1) == 113)
			break;

		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
