#include "marker.h"

using namespace cv;
using namespace std;
using namespace zbar;
namespace HMDETECTION{
MARKER::MARKER(ros::NodeHandle nh){
	nh_ = nh;
   // scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

}

void MARKER::subscriber(){
	ros::Subscriber imageSub = nh_.subscribe("/usb_cam/image_raw", 10, &MARKER::imageCallback, this);
	ros::spin();
}

void MARKER::imageCallback(const sensor_msgs::Image::ConstPtr& msg){

	cv_bridge::CvImageConstPtr cv_image;
	cv_image = cv_bridge::toCvShare(msg, "mono8");

	cv::Mat tmp=cv_image->image;
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
    /*for(int i=0; i<1; i++){
    	cv::pyrUp( singleImg, dst, cv::Size( tmp.cols*2, tmp.rows*2 ));
    	tmp = dst;
    }*/

    imshow("imgout.jpg",dst);
    waitKey(1);

//    zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data,cv_image->image.cols * cv_image->image.rows);
    zbar::Image zbar_image(tmp.cols, tmp.rows, "Y800", tmp.data,tmp.cols *tmp.rows);

    scanner_.scan(zbar_image);

    	// iterate over all barcode readings from image
    	for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol)
    	{
    		std::string barcode = symbol->get_data();
    		std::cout << barcode <<std::endl;
    	}

    	zbar_image.set_data(NULL, 0);
  //  	imshow("imgout.jpg",dst);
//    	  waitKey(1);
}

}

