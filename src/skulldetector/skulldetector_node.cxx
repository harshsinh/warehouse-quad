/*****************************************************************************
 *
 * Publishers : /lines -- geometry_msgs/Vector3 : x - slope, y - intercept
 * Publishers : /detected -- sensor_mags/Image  : detected Image with line
 * Subscriber : /usb_cam/image_raw -- Input Image.
 * Origin : (0, 0) at image center
 * x-axis is vertically up and y-axis is 90 deg counter-clockwise from x.
 *
 ****************************************************************************/
#include "CVInclude.h"
#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <geometry_msgs/Vector3.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#define TODEG 180/3.14

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
	ros::init (argc, argv, "skulldetector_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate (50);
	geometry_msgs::Vector3 pixelLine;
	image_transport::ImageTransport it(nh);

	// Publish the final skull detected image 
	image_transport::Publisher threshpub = it.advertise ("thresholded", 1);
	ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("/line", 100);
	image_transport::Subscriber sub = it.subscribe ("/usb_cam/image_raw", 1000, imcallback);

	//moving average
	// //int movcount = 0;
	// int movavglim = 3;
	// float movavgs = 0.0, movavgi = 0.0;

	//camera choice
	int camera = argv [1][0] - 48;

	// Camera has to specified in digit, else works on /usb_cam/image_raw
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
			
			cv::Mat  thresh, hsv, blurred, opening, closing, result, temp, final_image,storage;
			std::vector<cv::Vec4i> lines;

			cv::resize (frame, frame, cv::Size(256, 256));
			cv::cvtColor (frame, hsv, CV_BGR2HSV);
			cv::inRange (hsv, cv::Scalar(20, 80, 100), cv::Scalar(40, 255, 255), thresh);
			cv::GaussianBlur (thresh, blurred,  cv::Size(11, 11), 0, 0);
			//cv::threshold (blurred, thresh, 127, 255, CV_THRESH_BINARY);
			cv::morphologyEx (thresh, closing, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2),  cv::Point(-1, -1)));
			cv::morphologyEx (closing, opening, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2),  cv::Point(-1, -1)));
			cv::Canny (thresh, temp, 50, 150, 3);
			// cv::GaussianBlur (temp, temp,  cv::Size(11, 11), 0, 0);
			// cv::Canny (temp, temp, 50, 150, 3);

			result = cv::Scalar::all(0);
			temp.copyTo(result);

			cv::HoughLinesP (result, lines, 1, CV_PI/180, 15, 15);





 // IplImage* imgGrayScale = cvCreateImage(cvGetSize(frame), 8, 1); 
 // cvCvtColor(frame,imgGrayScale,CV_BGR2GRAY);

 // //thresholding the grayscale image to get better results
 // cvThreshold(imgGrayScale,imgGrayScale,128,255,CV_THRESH_BINARY);  
 
 // CvSeq* contours;  //hold the pointer to a contour in the memory block
 // CvSeq* result;   //hold sequence of points of a contour
 // CvMemStorage *storage = cvCreateMemStorage(0); //storage area for all contours
 
 //finding all contours in the image
 cvFindContours(opening, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
  
 //iterating through each contour
 while(contours)
 {
     //obtain a sequence of points of contour, pointed by the variable 'contour'
     result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0);
          
     //if there are 3  vertices  in the contour(It should be a triangle)
    if(result->total==3 )
     {
         //iterating through each point
         CvPoint *pt[3];
         for(int i=0;i<3;i++){
             pt[i] = (CvPoint*)cvGetSeqElem(result, i);
         }
   
         //drawing lines around the triangle
         cvLine(img, *pt[0], *pt[1], cvScalar(255,0,0),4);
         cvLine(img, *pt[1], *pt[2], cvScalar(255,0,0),4);
         cvLine(img, *pt[2], *pt[0], cvScalar(255,0,0),4);
       
     }

     //if there are 4 vertices in the contour(It should be a quadrilateral)
     else if(result->total==4 )
     {
         //iterating through each point
         CvPoint *pt[4];
         for(int i=0;i<4;i++){
             pt[i] = (CvPoint*)cvGetSeqElem(result, i);
         }
   
         //drawing lines around the quadrilateral
         cvLine(img, *pt[0], *pt[1], cvScalar(0,255,0),4);
         cvLine(img, *pt[1], *pt[2], cvScalar(0,255,0),4);
         cvLine(img, *pt[2], *pt[3], cvScalar(0,255,0),4);
         cvLine(img, *pt[3], *pt[0], cvScalar(0,255,0),4);   
     }

  //if there are 7  vertices  in the contour(It should be a heptagon)
     else if(result->total ==7  )
     {
         //iterating through each point
         CvPoint *pt[7];
         for(int i=0;i<7;i++){
             pt[i] = (CvPoint*)cvGetSeqElem(result, i);
         }
   
         //drawing lines around the heptagon
         cvLine(img, *pt[0], *pt[1], cvScalar(0,0,255),4);
         cvLine(img, *pt[1], *pt[2], cvScalar(0,0,255),4);
         cvLine(img, *pt[2], *pt[3], cvScalar(0,0,255),4);
         cvLine(img, *pt[3], *pt[4], cvScalar(0,0,255),4);
         cvLine(img, *pt[4], *pt[5], cvScalar(0,0,255),4);
         cvLine(img, *pt[5], *pt[6], cvScalar(0,0,255),4);
         cvLine(img, *pt[6], *pt[0], cvScalar(0,0,255),4);
     }

     //obtain the next contour
     contours = contours->h_next; 
 }

 //show the image in which identified shapes are marked   
 

			// std::cout << "******************" << std::endl;
			// std::cout << lines.size() << std::endl;

			// float temp_ = 0.0, slope = 0.0, interc = 0.0;
			// float slope_ = 0.0, interc_ = 0.0;
			// int count = 0;
			// movcount += 1;

			// for (std::vector<cv::Vec4i>::iterator i = lines.begin(); i != lines.end(); ++i) {

			// 	cv::Vec4i l = *i;
			// 	cv::line (frame, cv::Point (l[0], l[1]), cv::Point (l[2], l[3]), cv::Scalar(255, 0, 0), 3, CV_AA);
			// 	count += 1;

			// 	// Transformation from default coordinates
			// 	float x1_, y1_, x2_, y2_;
			// 	x1_ = 128 - l[1]; x2_ = 128 - l[3];
			// 	y1_ = 128 - l[0]; y2_ = 128 - l[2];

			// 	slope 	 = (y2_ - y1_)/((x2_ - x1_)?(x2_ - x1_):0.00001); // Prevent division by zero
			// 	interc   = slope*x1_ - y1_;

			// 	slope_  += slope;
			// 	interc_ += interc;

			// }

			// if (count) {

			// 	std::cout << "m : " << std::atan2(slope_, count)*TODEG << " c : " << (interc_/count) << std::endl;
			// 	movavgs += std::atan2(slope_, count);
			// 	movavgi += (interc_/count);
			// 	std::cout << "******************" << std::endl;

			// }

			// if (movcount == movavglim) {

			// 	pixelLine.x = movavgs/movavglim;
			// 	pixelLine.y = movavgi/movavglim;
			// 	movcount 	= 0;
			// 	movavgs 	= 0.0;
			// 	movavgi 	= 0.0;

			// }

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
