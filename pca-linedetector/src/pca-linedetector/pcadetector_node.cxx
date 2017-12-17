/*****************************************************************************
 * 
 * Publishers : /lines -- warehouse_quad::line
 *                        line.slope -- slope of major line
 *                        line.c1(c2)-- intercepts
 *                        line.mode  -- MODE of operation
 * Publishers : /detected -- sensor_mags/Image  : detected Image with line
 * Subscriber : /usb_cam/image_raw -- Input Image.
 * Origin : (0, 0) at image center
 * x-axis is vertically up and y-axis is 90 deg counter-clockwise from x.
 * 
 ****************************************************************************/
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include "CVInclude.h"
#include "warehouse_quad/line.h"
#include <geometry_msgs/Vector3.h>

#define min 0.00001

/* Frame and Camera */
cv::Mat frame;
cv::VideoCapture cap;

/* [TUNABLE] Color Thresh */
auto yellow_low  = cv::Scalar(20, 100, 100);
auto yellow_high = cv::Scalar(30, 255, 255);

/* Call back function for image */
void imcallback (const sensor_msgs::ImageConstPtr& msg)
{

    frame = cv_bridge::toCvShare (msg) -> image;
    return;

}

/* Transform from new coordinate system */
cv::Point transform (double x, double y)
{

    int x_ = 128 - y;
    int y_ = 128 - x;

    return cv::Point(x_, y_);
}

/* PCA : returns lines along first and second principle components */
cv::Vec4f PCA (std::vector<cv::Vec2i> &data_points)
{

    /* Safe gaurd for empty frames */
    if(data_points.size() == 0)
        return (cv::Vec4i(-1, -1, -1, -1));

    cv::Vec4f lines;
    auto mean = cv::mean(data_points);
    cv::Mat data_transp(2, data_points.size(), CV_64F, cv::Scalar::all(0));
    cv::Mat data(data_points.size(), 2, CV_64F, cv::Scalar::all(0));
    
    /* Data formatting for PCA */
    int i = 0;
    for (std::vector<cv::Vec2i>::iterator it = data_points.begin(); it != data_points.end(); ++it) {

        auto d = *it;
        data.at<double>(i, 0) = d[0] - mean[0];
        data.at<double>(i, 1) = d[1] - mean[1];
        data_transp.at<double>(0, i) = d[0] - mean[0];
        data_transp.at<double>(1, i) = d[1] - mean[1];
        ++i;
    }

    cv::Mat cov = data_transp * data;
    cv::Mat eigenval, eigenvec;
    cv::eigen(cov, eigenval, eigenvec);
    
    for(int i = 0; i < eigenvec.rows; ++i) {

        double * veci = eigenvec.ptr<double>(i);

        /* Angle for the vector constrained within 0-180 */
        if (veci[1] < 0) {

            veci[0] = -veci[0];
            veci[1] = -veci[1];
        }

        /* Slope and Intercept calculation */
        lines[2*i] = veci[1]/(veci[0]?veci[0]:min);
        lines[2*i + 1] = mean[1] - (lines[2*i] * mean[0]);

        std::cout << i << std::endl;

    }

    std::cout << lines << std::endl;
    std::cout << "slope major : " << std::atan2(lines[0], 1) * 180/3.1415 << "\t"
              << "inter : " << lines[1] << std::endl;
    std::cout << "slope minor : " << std::atan2(lines[2], 1) * 180/3.1415 << "\t"
              << "inter : " << lines[3] << std::endl;
    return (lines);
}

int main (int argc, char** argv)
{

    /* ROS Node objects */
	ros::init (argc, argv, "linedetector_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate (50);
    warehouse_quad::line pixelLine;
	image_transport::ImageTransport it(nh);
    int msg_count = -1;
	
	/* Publish the final line detected image and line */
	image_transport::Publisher threshpub = it.advertise ("thresholded", 1);
	ros::Publisher pub = nh.advertise<warehouse_quad::line>("/line", 100);
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

            cv::HoughLinesP(result, lines, 5, CV_PI/180, 1, 1);

            /* PCA on the point from Hough lines */
            for (std::vector<cv::Vec4i>::iterator it = lines.begin(); it != lines.end(); ++it) {

                cv::Vec4i l = *it;

                // cv::line (frame, cv::Point (l[0], l[1]), cv::Point (l[2], l[3]), cv::Scalar(255, 0, 0), 3, CV_AA);
                
                // Transformation from default coordinates
				int x1_, y1_, x2_, y2_;
				x1_ = 128 - l[1]; x2_ = 128 - l[3];
                y1_ = 128 - l[0]; y2_ = 128 - l[2];
                
                points.push_back(cv::Vec2i(x1_, y1_));
                points.push_back(cv::Vec2i(x2_, y2_));

            }

            auto principle_lines = PCA (points);

            auto m1_ = principle_lines[0];
            auto c1_ = principle_lines[1];
            auto m2_ = principle_lines[2];
            auto c2_ = principle_lines[3];

            /* Slopes of the two principle components can not be same */
            if(m1_ != m2_) {

                cv::line(frame, transform(0, c1_), transform(-c1_/m1_?m1_:min, 0), cv::Scalar(0, 0, 255), 2);
                cv::line(frame, transform(0, c2_), transform(-c2_/m2_?m2_:min, 0), cv::Scalar(0, 0, 255), 2);

                pixelLine.header.seq = ++msg_count;
                pixelLine.header.stamp = ros::Time::now();
                pixelLine.header.frame_id = "0";
                pixelLine.slope = m1_;
                pixelLine.c1 = c1_;
                pixelLine.c2 = 0;
                pixelLine.mode = 1;
            }

            cv::imshow("opening", opening);
            cv::imshow("canny", result);
            cv::imshow("image", frame);

            pub.publish(pixelLine);

            if (cv::waitKey(1) == 113)
			break;

        }

        ros::spinOnce();
	    loop_rate.sleep();
    }

    return 0;
}