#include "marker.h"

namespace HMDETECTION{
MARKER::MARKER(ros::NodeHandle nh){
	nh_ = nh;
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

}

void MARKER::subscriber(){
	ros::Subscriber imageSub = nh_.subscribe("/usb_cam/image_raw", 10, &MARKER::imageCallback, this);
	ros::spin();
}

void MARKER::imageCallback(const sensor_msgs::Image::ConstPtr& msg){

	cv_bridge::CvImageConstPtr cv_image;
	cv_image = cv_bridge::toCvShare(msg, "mono8");

	zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data,cv_image->image.cols * cv_image->image.rows);
	scanner_.scan(zbar_image);

	// iterate over all barcode readings from image
	for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol)
	{
		std::string barcode = symbol->get_data();
		std::cout << barcode <<std::endl;
	}

	zbar_image.set_data(NULL, 0);

}

}

