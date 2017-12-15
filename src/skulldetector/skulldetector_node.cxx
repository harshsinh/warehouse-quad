


#include "CVInclude.h"
#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <geometry_msgs/Vector3.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace std;

VideoCapture cap;

void drawStuff();
void drawAllTriangles(Mat&, const vector< vector<Point> >&);

Mat img_rgb,img_gray,canny_output,drawing,blurred,thresh1,closing,opening,temp;
void imcallback (const sensor_msgs::ImageConstPtr& msg)
{

	img_rgb = cv_bridge::toCvShare (msg) -> image;
	return;

}

int thresh = 100;
int max_thresh = 255;

int main (int argc, char** argv)
{

	//ros node stuff
	ros::init (argc, argv, "skulldetector_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate (50);
	 geometry_msgs::Vector3 pixelLine;
	image_transport::ImageTransport it(nh);

	// Publish the final line detected image and line
	 image_transport::Publisher threshpub = it.advertise ("thresholded", 1);
	ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("/line", 100);
	image_transport::Subscriber sub = it.subscribe ("/usb_cam/image_raw", 1000, imcallback);

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

while (nh.ok()){

  if (cap.isOpened())
    cap >> img_rgb;

  if (!img_rgb.empty()) {


    // cvtColor(img_rgb,img_gray,CV_RGB2GRAY);
     cvtColor (img_rgb, img_gray, CV_BGR2HSV);
     inRange (img_gray,  Scalar(20, 80, 100),  Scalar(40, 255, 255), thresh1);
     GaussianBlur (thresh1, blurred,   Size(11, 11), 0, 0);
      threshold (blurred, thresh1, 127, 255, CV_THRESH_BINARY);
     morphologyEx (thresh1, closing,  MORPH_CLOSE,  getStructuringElement( MORPH_RECT,  Size(2, 2),   Point(-1, -1)));
     morphologyEx (closing, opening,  MORPH_OPEN,  getStructuringElement( MORPH_RECT,  Size(2, 2),   Point(-1, -1)));
     Canny (thresh1, temp, 50, 150, 3);
    imshow("InputImage",img_rgb);
    drawStuff();
    if (cv::waitKey(1) == 113)
    break;
  }
    ros::spinOnce();
    loop_rate.sleep();
    }
return 0;

}

void drawStuff(){
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Canny( img_gray, canny_output, thresh, thresh*2, 3 );
    imshow("Canny",temp);
    findContours( temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    drawing = Mat::zeros( temp.size(), CV_8UC3 );

    drawAllTriangles(drawing,contours);
    imshow("Triangles",drawing);
}

void drawAllTriangles(Mat& img, const vector< vector<Point> >& contours){
    vector<Point> approxTriangle;
    for(size_t i = 0; i < contours.size(); i++){
        approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);
        if(approxTriangle.size() == 3){
            drawContours(img, contours, i, Scalar(0, 255, 255), CV_FILLED); // fill GREEN
            vector<Point>::iterator vertex;
            for(vertex = approxTriangle.begin(); vertex != approxTriangle.end(); ++vertex){
                circle(img, *vertex, 3, Scalar(0, 0, 255), 1);
								std::cout << " Area: " << contourArea(contours[i]) << std::endl;

            }
        }
    }
}
