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
#include "CVInclude.h"
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/Bool.h>
#include "hemd/line.h"
#include "hemd/markerInfo.h"
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>

#define min 0.00001
#define EIGMIN 0.005*1e6
#define CROSS_THRESH 80
#define ERROR_VAL 1000
#define n_grid 1
#define ZED 0
#define SIZE 256
#define TO_PI 180/3.1415

using namespace Eigen;

/* Necessary Globals */
VectorXf sonar_set(10);
double sonarVal;
double threshold = 20/(TO_PI);
double set_count = 0;
int first_val_check = 0;
bool follow = false;
bool turn = false;
bool pass = true;
bool flag = false;
double height = 0;

/* Frame and Camera */
cv::Mat frame;
cv::VideoCapture cap;

/* [TUNABLE] Color Thresh */
auto yellow_low  = cv::Scalar(20, 80, 155);
auto yellow_high = cv::Scalar(40, 255, 255);

/* Call back function for image */
void imcallback (const sensor_msgs::ImageConstPtr& msg)
{

    frame = cv_bridge::toCvShare (msg) -> image;
    return;

}

/* Call back for detection message */
void follow_again (const std_msgs::Bool& msg)
{

    follow = msg.data;
    return;
}

/* Call back for markers */
void marker_cb (const hemd::markerInfo& msg)
{

    turn = ((msg.col == 4) && (msg.shelf == 1));
    return;
}

/* Call back for height */
void height_cb (const geometry_msgs::PoseStamped& msg)
{

	height = msg.pose.position.z;
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
    // std::cout << "number of points : " << data_points.size() << std::endl;
  
    if(data_points.size() == 0)
        return (cv::Vec4i(-ERROR_VAL, -ERROR_VAL, -ERROR_VAL, -ERROR_VAL));

    cv::Vec4f lines;
    std::vector<double> values;
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
        double * eigval = eigenval.ptr<double>(i);

        /* Angle for the vector constrained within 0-180 */
        if (veci[1] < 0) {

            veci[0] = -veci[0];
            veci[1] = -veci[1];
        }

        /* Slope and Intercept calculation */
        lines[2*i] = veci[1]/(veci[0]?veci[0]:min);
        lines[2*i + 1] = mean[1] - (lines[2*i] * mean[0]);

        values.push_back(eigval[0]);
    }

//    std::cout << "eigen val : " << values[0] << " : " << values[1] << std::endl;

    if (values[0] < EIGMIN)
        return (cv::Vec4i(-ERROR_VAL, -ERROR_VAL, -ERROR_VAL, -ERROR_VAL));

    // std::cout << lines << std::endl;
    // std::cout << "slope major : " << std::atan2(lines[0], 1) * 180/3.1415 << "\t"
    //           << "inter : " << lines[1] << std::endl;
    // std::cout << "slope minor : " << std::atan2(lines[2], 1) * 180/3.1415 << "\t"
    //           << "inter : " << lines[3] << std::endl;

    lines[2] = values[0];
    lines[3] = values[1];
    return (lines);
}

bool median_filter (double distance)
{

    int i;
    //double set_count = 0.0;
	VectorXf sonar_set_copy(10);

	if (set_count==0){
		sonar_set.setZero();
	}
	sonar_set_copy = sonar_set;
	if (set_count < 10){
		//getting first 40 set of sonar data
		sonar_set_copy[set_count] = distance;
		sonar_set[set_count] = distance;
		set_count++;
		sonarVal = distance;
		if(set_count==9){
			first_val_check = 1;
		}
	}

	else{

		    std::sort(sonar_set_copy.data(), sonar_set_copy.data()+sonar_set_copy.size()); //arranging the 400 set of data in increasing order

		    if((distance > (sonar_set_copy[5]-threshold))&&(distance < (sonar_set_copy[5]+threshold))){ //checking if the current value is in a limit of the median value
			    //if so then return the current sonar data
			    sonarVal = distance;
			    for(i=0;i<9;i++){
			    sonar_set[i] = sonar_set[i+1]; // updating the sonar set for next 400 data
			}

    		sonar_set[9] = distance;
		    return true;
		}
		
        else{

    		sonar_set[9] = distance;
		    return false;
		}
	}

    return false;
}

int main (int argc, char** argv)
{

    /* ROS Node objects */
	ros::init (argc, argv, "linedetector_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate (50);
    hemd::line pixelLine;
    geometry_msgs::Vector3 debug_msg;
	image_transport::ImageTransport it(nh);
    sensor_msgs::ImagePtr threshmsg, finalmsg;
    int msg_count = -1;
    int cnts = 0;

	/* Publish the final line detected image and line */
	image_transport::Publisher threshpub = it.advertise ("thresh", 1);
	image_transport::Publisher finalimpub = it.advertise ("final_image", 1);

	ros::Publisher pub = nh.advertise<hemd::line>("/warehouse_quad/line", 100);
	ros::Publisher debug = nh.advertise<geometry_msgs::Vector3>("/debug", 100);

	image_transport::Subscriber sub = it.subscribe ("/usb_cam/image_raw", 1000, imcallback);
	ros::Subscriber detect_sub = nh.subscribe ("/warehouse_quad/follow_line", 1, follow_again);
	ros::Subscriber marker_sub = nh.subscribe ("/warehouse_quad/marker", 1, marker_cb);
	ros::Subscriber height_sub = nh.subscribe ("/mavros/vision_pose/pose", 100, height_cb);


    /* Choose camera */
	int camera = argv [1][0] - 48;

    /* Parameters for Regular Slices */
    auto image_size = cv::Size(SIZE, SIZE);

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
            std::vector<cv::Vec2i> all_points;
            std::vector<cv::Vec2i> points[2*n_grid];
            std::vector<cv::Vec3f> h_grid;
            std::vector<cv::Vec3f> v_grid;
            std::vector<cv::Vec2i> X, Y0, Y1;

            /* Slice Image in case of ZED camera */
            if (ZED) {
                frame = cv::Mat(frame, cv::Rect(0, 0, frame.cols/2, frame.rows)).clone();
            }

            /* Load, Resize and Convert image */
            cv::resize(frame, frame, image_size);
            cv::cvtColor(frame, hsv, CV_BGR2HSV);
            cv::inRange(hsv, yellow_low, yellow_high, thresh);
			cv::GaussianBlur (thresh, blurred,  cv::Size(11, 11), 0, 0);
			cv::morphologyEx (blurred, closing, cv::MORPH_CLOSE,
                              cv::getStructuringElement (cv::MORPH_RECT,
                              cv::Size(2, 2),  cv::Point(-1, -1)));

            cv::morphologyEx (closing, opening, cv::MORPH_OPEN,
                              cv::getStructuringElement (cv::MORPH_RECT,
                              cv::Size(2, 2),  cv::Point(-1, -1)));

            cv::Canny (thresh, result, 50, 150, 3);
            cv::HoughLinesP(result, lines, 5, CV_PI/180, 1, 4);

            threshmsg = cv_bridge::CvImage (std_msgs::Header(), "mono8", result).toImageMsg();

            /* PCA on the point from Hough lines */
            for (std::vector<cv::Vec4i>::iterator it = lines.begin(); it != lines.end(); ++it) {

                cv::Vec4i l = *it;
                
        // Transformation from default coordinates
				int x1_, y1_, x2_, y2_;
				x1_ = 128 - l[1]; x2_ = 128 - l[3];
                y1_ = 128 - l[0]; y2_ = 128 - l[2];

		/* Points for Vertical Line stored here */
                if (std::abs(y2_ - y1_) <= std::abs(x2_ - x1_)) {

                    cv::line (frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 2, CV_AA);
                    X.push_back (cv::Vec2i(x1_, y1_));
                    X.push_back (cv::Vec2i(x2_, y2_));
		}

		/* Points for Horizontal Line stored in Y0 and Y1 stores line as buffer if any */
                else {

			if (std::abs(x1_) < CROSS_THRESH && std::abs(x2_) < CROSS_THRESH) {

                    		cv::line (frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, CV_AA);
				Y0.push_back (cv::Vec2i(y1_, x1_));
				Y0.push_back (cv::Vec2i(y2_, x2_));
                	}

			else if (x1_ > CROSS_THRESH && x2_ > CROSS_THRESH) {

				Y1.push_back (cv::Vec2i(y1_, x1_));
				Y1.push_back (cv::Vec2i(y2_, x2_));
			}
		}

                all_points.push_back(cv::Vec2i(x1_, y1_));
                all_points.push_back(cv::Vec2i(x2_, y2_));

            }

            auto principle_lines_h = PCA (Y0);
            auto principle_lines_v = PCA (X);
		auto buffer_line_h = PCA (Y1);	

            auto m1_ = principle_lines_v[0];
            auto c1_ = principle_lines_v[1];
            auto m2_ = principle_lines_h[0];
            auto c2_ = principle_lines_h[1];
		auto m_buff = buffer_line_h[0];
		auto c_buff = buffer_line_h[1];	

            if (m1_ != -ERROR_VAL) {

		/* Putting a cap on things */
                if (std::abs(m1_ * TO_PI) > 90)
                    m1_ = CV_PI * 0.5 * ((m1_ > 0) ? 1 : -1);

		/* Putting a cap on things */
                if (std::abs(c1_) > 128)
                    c1_ = 128 * ((c1_ > 0) ? 1 : -1);

		pass =	median_filter(m1_);

                pixelLine.header.seq = ++msg_count;
                pixelLine.header.stamp = ros::Time::now();
                pixelLine.header.frame_id = "0";
                pixelLine.slope = m1_;
		pixelLine.c1 = c1_;
                pixelLine.c2 = 0;
                pixelLine.mode = 1;
                cnts = 0;

                cv::line(frame, transform(0, c1_), transform(-c1_/(m1_?m1_:min), 0), cv::Scalar(255, 255, 0), 2);
            }

            else {

                    ++cnts;
                    if (cnts < 5)
                        continue;

                    pixelLine.header.seq = ++msg_count;
                    pixelLine.header.stamp = ros::Time::now();
                    pixelLine.header.frame_id = "0";
                    pixelLine.slope = 0;
                    pixelLine.c1 = 0;
                    pixelLine.c2 = 0;
                    pixelLine.mode = 0;
                }

/******************************* Redundant Code ***************************/
/* TODO: Depracate this */
            if (m2_ != -ERROR_VAL) {

		/* Putting a cap on things */
                if (std::abs(c2_) > 128)
                    c2_ = 128 * ((c2_ > 0) ? 1 : -1);

                pixelLine.c2 = c2_;

                cv::line(frame, transform(c2_, 0), transform(0, -c2_/(m2_?m2_:min)), cv::Scalar(255, 0, 255), 2);
            }
/*************************************************************************/

		/* Draw the intersection point */
            if (m1_ != -ERROR_VAL && m2_ != -ERROR_VAL)
                cv::circle(frame, transform((m2_*c1_ + c2_)/(1 - m1_*m2_), (m1_*c2_ + c1_)/(1 - m1_*m2_)), 5, cv::Scalar(0, 255, 0), 5);

            if (m1_ != -ERROR_VAL && m2_ != -ERROR_VAL && std::abs(c2_) < CROSS_THRESH) {

                /* If the marker_detected is high, force move to line follow mode */
                pixelLine.mode = 2;

		/* Putting a cap on things */
                if (std::abs(c2_) > 128)
                    c2_ = 128 * ((c2_ > 0) ? 1 : -1);

                pixelLine.c1 = c1_;
                pixelLine.c2 = c2_;

		/* If markers are detcted take orders from the line in Y1 */
		if (follow)
			flag = true;

		if (flag) {

			pixelLine.mode = 1;
			if ((c_buff < (CROSS_THRESH + 10)) && c_buff != -ERROR_VAL)
				flag = false;
		}
            }

        }

        finalmsg = cv_bridge::CvImage (std_msgs::Header(), "bgr8", frame).toImageMsg();

        debug_msg.x = pixelLine.slope * 180/3.14159;
	/* This is ensure that the tunnings do not change as height is changed */
	pixelLine.c1 = pixelLine.c1 * (height/0.7);


	if (pass)
		pub.publish(pixelLine);
        threshpub.publish(threshmsg);
        finalimpub.publish(finalmsg);
        debug.publish(debug_msg);

        if (cv::waitKey(1) == 113)
		break;

        ros::spinOnce();
	    loop_rate.sleep();
    }

    return 0;
}
