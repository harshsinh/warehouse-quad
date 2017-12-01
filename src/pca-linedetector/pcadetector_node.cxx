#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include "CVInclude.h"
#include "slic.h"
#include <geometry_msgs/Vector3.h>

/* Frame and Camera */
cv::Mat frame;
cv::VideoCapture cap;

/* Colors */
auto yellow_low  = cv::Scalar(20, 100, 100);
auto yellow_high = cv::Scalar(30, 255, 255);

/* Call back function for image */
void imcallback (const sensor_msgs::ImageConstPtr& msg)
{

    frame = cv_bridge::toCvShare (msg) -> image;
    return;

}

/* PCA : returns lines along first and second principle components */
cv::Vec4i PCA (std::vector<cv::Vec2i> &data_points)
{

    auto mean = cv::mean(data_points);
    std::vector<cv::Vec2i> data;
    for (std::vector<cv::Vec2i>::iterator it = data_points.begin(); it != data_points.end(); ++it) {

        auto d = *it;
        data.push_back(cv::Vec2i(d[0] - mean[0], d[1] - mean[1]));
    }
    
    std::cout << mean[0] << mean[1] << std::endl;
    // data_points = data_points - cv::Vec2i(mean[0], mean[1]);
    // std::cout << mean << std::endl;
    return (cv::Vec4i(1, 2, 3, 4));
}

int main (int argc, char** argv)
{

    /* ROS Node objects */
	ros::init (argc, argv, "linedetector_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate (50);
	geometry_msgs::Vector3 pixelLine;
	image_transport::ImageTransport it(nh);
	
	/* Publish the final line detected image and line */
	image_transport::Publisher threshpub = it.advertise ("thresholded", 1);
	ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("/line", 100);
	image_transport::Subscriber sub = it.subscribe ("/usb_cam/image_raw", 1000, imcallback);

    /* Choose camera */
	int camera = argv [1][0] - 48;

    /* Parameters for Regular Grid */
    int n_grid = 16;
    auto image_size = cv::Size(256, 256);
    auto grid_size = image_size/n_grid;

    /* Image size, Parameters for SLIC */
    int nr_superpixels = 100;
    int nc = 25;

	/* Camera has to specified in digit, else checks on /usb_cam/image_raw */
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

            /* Empty Images and Lines */
            cv::Mat hsv, thresh, blurred, opening, closing, result, temp, final_image;
            std::vector<cv::Vec4i> lines;
            std::vector<cv::Vec2i> points;

            /* Load, Resize and Convert image */
            cv::resize(frame, frame, image_size);
            cv::cvtColor(frame, hsv, CV_BGR2HSV);
            cv::inRange(hsv, yellow_low, yellow_high, thresh);
			cv::GaussianBlur (thresh, blurred,  cv::Size(11, 11), 0, 0);
			cv::morphologyEx (blurred, closing, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2),  cv::Point(-1, -1)));
			cv::morphologyEx (closing, opening, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2),  cv::Point(-1, -1)));
            cv::Canny (thresh, result, 50, 150, 3);

            cv::HoughLinesP(result, lines, 5, CV_PI/180, 10, 10);

            /* PCA on the point from Hough lines */
            int i = 0;
            for (std::vector<cv::Vec4i>::iterator it = lines.begin(); it != lines.end(); ++it) {

                cv::Vec4i l = *it;
                // points.push_back(cv::Vec2i(l[0], l[1]));
                // points.push_back(cv::Vec2i(l[2], l[3]));
                points.push_back(cv::Vec2i(i, i + 1));
                ++i;
            }

            auto principle_lines = PCA (points);

/*******************************************************************/
/* SLIC not good, fails on mean calculation */
            /*auto lab_image = frame.clone();
            cv::cvtColor (frame, lab_image, CV_BGR2Lab);*/
            
            /* Number of super-pixels and weight factors *
            int w = frame.rows, h = frame.cols;
            double step = std::sqrt((w * h) / nr_superpixels);

            /* Perform SLIC *
            Slic slic;
            slic.generate_superpixels (lab_image, step, nc);
            slic.create_connectivity (lab_image);

            /* Display the results *
            slic.display_contours(frame, cv::Vec3b (0, 0, 255));
/*******************************************************************/

            cv::imshow("image", frame);
            cv::imshow("opening", opening);
            cv::imshow("canny", result);

            if (cv::waitKey(1) == 113)
			break;

        }

        ros::spinOnce();
	    loop_rate.sleep();
    }

    return 0;
}