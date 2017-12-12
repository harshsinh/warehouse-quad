#include "marker.h"

using namespace cv;
using namespace std;
using namespace zbar;
namespace HMDETECTION{
MARKER::MARKER(ros::NodeHandle nh){
	nh_ = nh;
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    Mat tmp, frame;
       VideoCapture cap;
       if(!cap.open("http://192.168.0.107:8080/video?x.mjpeg"))
       {
           cout<<"Camera not found"<<endl;
           return;
       }
       while ( cap.isOpened() )
       {
           cap >> tmp;
           if(tmp.empty()) break;


    cvtColor(tmp, frame, cv::COLOR_RGB2GRAY);

   	/*cv_bridge::CvImageConstPtr cv_image;
   	cv_image = cv_bridge::toCvShare(msg, "mono8");

   	cv::Mat tmp=cv_image->image;

   	float rescale_value=1.2;
   	cv::Mat sig;
   	cv::resize(tmp, sig, cvSize(0, 0), rescale_value, rescale_value);*/

   	zbar::Image zbar_image(frame.cols, frame.rows, "Y800", frame.data,frame.cols * frame.rows);

   	scanner_.scan(zbar_image);


   	// iterate over all barcode readings from image
   	for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol)
   	{

   		std::string barcode = symbol->get_data();
   		std::cout << barcode <<std::endl;

   	}
   	imshow("imgout.jpg",frame);
   	waitKey(1);
       }

}

void MARKER::subscriber(){
	ros::Subscriber imageSub = nh_.subscribe("/usb_cam/image_raw", 10, &MARKER::imageCallback, this);
	ros::spin();

}

void MARKER::imageCallback(const sensor_msgs::Image::ConstPtr& msg){




	/*cv::Mat tmp=cv_image->image;
    //crop the stereo image to single image
    cv::Rect roi;
    roi.x = 0;
    roi.y = 0;
    roi.width = tmp.cols/2;
    roi.height = tmp.rows;

   // cv::Mat sig = tmp(roi);
    cv::Mat sig = tmp(roi);

    roi.x = sig.cols/4;
    roi.y = sig.rows/4;
    roi.width = sig.cols/2;
    roi.height = sig.rows/2;
    cv::Mat singleImg = tmp(roi);


    cv::Mat dst=singleImg;
    for(int i=0; i<1; i++){
    	cv::pyrUp( singleImg, dst, cv::Size( tmp.cols*2, tmp.rows*2 ));
    	tmp = dst;
    }

    imshow("imgout.jpg",dst);
    waitKey(1);*/

}

}

