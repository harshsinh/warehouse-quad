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
#include "pca_linedetector/line.h"
#include <geometry_msgs/Vector3.h>

#define min 0.00001
#define EIGMIN 0.25*1e6
#define ERROR_VAL 1000
#define n_grid 3
#define ZED 0
#define SIZE 256
#define TO_PI 180/3.1415

/* Frame and Camera */
cv::Mat frame;
cv::VideoCapture cap;

/* [TUNABLE] Color Thresh */
auto yellow_low  = cv::Scalar(20, 80, 100);
auto yellow_high = cv::Scalar(40, 255, 255);

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

    if (values[0] < EIGMIN)
        return (cv::Vec4i(-ERROR_VAL, -ERROR_VAL, -ERROR_VAL, -ERROR_VAL));

    // std::cout << lines << std::endl;
    // std::cout << "slope major : " << std::atan2(lines[0], 1) * 180/3.1415 << "\t"
    //           << "inter : " << lines[1] << std::endl;
    // std::cout << "slope minor : " << std::atan2(lines[2], 1) * 180/3.1415 << "\t"
    //           << "inter : " << lines[3] << std::endl;

    // std::cout << "eigen val : " << values[0] << " : " << values[1] << std::endl;

    return (lines);
}

std::pair<int, int> get_index (int x, int y, int w)
{

    std::pair<int, int> v = {0, 0};

    v.first = (n_grid - 1)/2 + double(x + w/2)/w;
    v.second = (n_grid - 1)/2 + double(y + w/2)/w;

    if (x == SIZE/2)
        v.first = n_grid - 1;

    if (y == SIZE/2)
        v.second = n_grid - 1;
    return v;
}

int main (int argc, char** argv)
{

    /* ROS Node objects */
	ros::init (argc, argv, "linedetector_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate (50);
    pca_linedetector::line pixelLine;
    geometry_msgs::Vector3 debug_msg;
	image_transport::ImageTransport it(nh);
    sensor_msgs::ImagePtr msg;
    int msg_count = -1;
    int cnts = 0;

	/* Publish the final line detected image and line */
	image_transport::Publisher threshpub = it.advertise ("final_image", 1);
	ros::Publisher pub = nh.advertise<pca_linedetector::line>("/line", 100);
    ros::Publisher debug = nh.advertise<geometry_msgs::Vector3>("/debug", 100);
	image_transport::Subscriber sub = it.subscribe ("/usb_cam/image_raw", 1000, imcallback);

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
            std::vector<cv::Vec2i> X, Y;

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

            cv::HoughLinesP(result, lines, 5, CV_PI/180, 1, 1);

            // cv::imshow("opening", opening);
            // cv::imshow("canny", result);
            // cv::imshow("image", frame);
            msg = cv_bridge::CvImage (std_msgs::Header(), "mono8", result).toImageMsg();

            /* PCA on the point from Hough lines */
            for (std::vector<cv::Vec4i>::iterator it = lines.begin(); it != lines.end(); ++it) {

                cv::Vec4i l = *it;

                // Transformation from default coordinates
				int x1_, y1_, x2_, y2_;
				x1_ = 128 - l[1]; x2_ = 128 - l[3];
                y1_ = 128 - l[0]; y2_ = 128 - l[2];

                if (std::abs(y2_ - y1_) <= std::abs(x2_ - x1_)) {

                    cv::line (frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 2, CV_AA);
                    X.push_back (cv::Vec2i(x1_, y1_));
                    X.push_back (cv::Vec2i(x2_, y2_));
                }

                else {

                    cv::line (frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, CV_AA);
                    Y.push_back (cv::Vec2i(y1_, x1_));
                    Y.push_back (cv::Vec2i(y2_, x2_));
                }

                /* horizontal slice at index and vertical at index + n_grid */
                points[get_index(x1_, y1_, SIZE/n_grid).first].push_back(cv::Vec2i(y1_, x1_));
                points[get_index(x1_, y1_, SIZE/n_grid).second + n_grid].push_back(cv::Vec2i(x1_, y1_));

                points[get_index(x2_, y2_, SIZE/n_grid).first].push_back(cv::Vec2i(y2_, x2_));
                points[get_index(x2_, y2_, SIZE/n_grid).second + n_grid].push_back(cv::Vec2i(x2_, y2_));

                all_points.push_back(cv::Vec2i(x1_, y1_));
                all_points.push_back(cv::Vec2i(x2_, y2_));

            }

            for (int i = 0; i < n_grid; ++i) {


                std::cout << "---------------" << std::endl;
                auto principle_lines_h = PCA (X);
                auto principle_lines_v = PCA (Y);

                auto m1_ = principle_lines_h[0];
                auto c1_ = principle_lines_h[1];
                auto m2_ = principle_lines_v[0];
                auto c2_ = principle_lines_v[1];

                if (m1_ != -ERROR_VAL) {

                    pixelLine.c2 = c1_;

                    h_grid.push_back (cv::Vec3f(m1_, c1_, i));
                    cv::line(frame, transform(0, c1_), transform(-c1_/(m1_?m1_:min), 0), cv::Scalar(0, 255, 255), 2);
                    std::cout << "horizontal : " << i << " slope : " << m1_ * TO_PI << " intercept : " << c1_ << std::endl;
                }

                if (m2_ != -ERROR_VAL) {

                    pixelLine.header.seq = ++msg_count;
                    pixelLine.header.stamp = ros::Time::now();
                    pixelLine.header.frame_id = "0";
                    pixelLine.slope = m2_;
                    pixelLine.c1 = c2_;
                    pixelLine.mode = 1;

                    v_grid.push_back (cv::Vec3f(m2_, c2_, i));
                    cv::line(frame, transform(c2_, 0), transform(0, -c2_/(m2_?m2_:min)), cv::Scalar(0, 255, 255), 2);
                    std::cout << "vertical : " << i << " slope : " << m2_ * TO_PI << " intercept : " << c2_ << std::endl;
                }

                if (m1_ != -ERROR_VAL && m2_ != -ERROR_VAL) {

                    cv::circle(frame, transform((c1_ + c2_)/2, (c2_*m1_ + c1_)/2), 5, cv::Scalar(0, 255, 0), 5);
                }

            }

            cv::imshow("lol", frame);
            // if (v_grid.empty()) {

            //     auto principle_lines = PCA (all_points);
            //     auto m1_ = principle_lines[0];
            //     auto c1_ = principle_lines[1];
                
            //     if (m1_ != -ERROR_VAL) {

            //         pixelLine.header.seq = ++msg_count;
            //         pixelLine.header.stamp = ros::Time::now();
            //         pixelLine.header.frame_id = "0";
            //         pixelLine.slope = m1_;
            //         pixelLine.c1 = c1_;
            //         pixelLine.c2 = 0;
            //         pixelLine.mode = 1;

            //         std::cout << "slope : " << m1_ * TO_PI << "intercept : " << c1_ << std::endl;
            //     }

                // else {

                //     ++cnts;
                //     if (cnts < 10)
                //         continue;

                //     pixelLine.header.seq = ++msg_count;
                //     pixelLine.header.stamp = ros::Time::now();
                //     pixelLine.header.frame_id = "0";
                //     pixelLine.slope = m1_;
                //     pixelLine.c1 = 0;
                //     pixelLine.c2 = 0;
                //     pixelLine.mode = 0;
                // }
            // }
            // auto principle_line = PCA (all_points);
            // // auto principle_line_x = PCA (X);

            // auto m1_ = principle_line[0];
            // auto c1_ = principle_line[1];
            // if (m1_ != ERROR_VAL)
            //     std::cout << "m1 : " << m1_ * TO_PI << " c1 : " << c1_ << std::endl;
            // std::cout << "--------" << std::endl;

            std::cout << "****" << std::endl;
            pub.publish(pixelLine);
            threshpub.publish(msg);

            if (cv::waitKey(1) == 113)
			break;

        }

        ros::spinOnce();
	    loop_rate.sleep();
    }

    return 0;
}