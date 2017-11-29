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

/* Call back function for image */
void imcallback (const sensor_msgs::ImageConstPtr& msg)
{

    frame = cv_bridge::toCvShare (msg) -> image;
    return;

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

    /* Image size, Parameters for SLIC */
    auto image_size = cv::Size(256, 256);
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

            /* Load, Resize and Convert image */
            cv::resize (frame, frame, image_size);
            auto lab_image = frame.clone();
            auto mean_image = frame.clone();
            cv::cvtColor (frame, lab_image, CV_BGR2Lab);

/*******************************************************************/
/* SLIC not good, fails on mean calculation */
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
            pixelLine.x = 1;
            pub.publish(pixelLine);

            if (cv::waitKey(1) == 113)
			break;

        }

        ros::spinOnce();
	    loop_rate.sleep();
    }

    return 0;
}