#include "marker.h"

using namespace cv;
using namespace std;
using namespace zbar;
namespace HMDETECTION{
MARKER::MARKER(){
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    Mat tmp, frame;
    VideoCapture cap;

    ros::NodeHandle nh_;
	markerPub = nh_.advertise<warehouse_quad::markerInfo>("/marker",10);

	bool check=cap.open("http://192.168.42.129:8080/video?x.mjpeg");

    if(!check)
    {
    	cout<<"Camera not found"<<endl;
    	return;
    }
    while ( cap.isOpened()& ros::ok() )
    {
    	cap >> tmp;
    	if(tmp.empty()) break;

        cv::Rect roi;
        roi.x = tmp.rows/4;
        roi.y = tmp.cols/4;
        roi.width = tmp.cols/2;
        roi.height = tmp.rows/2;

        cv::Mat crop = tmp(roi);

    	cvtColor( crop, frame, CV_BGR2GRAY );


    	zbar::Image zbar_image(frame.cols, frame.rows, "Y800", frame.data,frame.cols * frame.rows);
    	scanner_.scan(zbar_image);

    	// iterate over all barcode readings from image
    	for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol)
    	{

    		std::string barcode = symbol->get_data();
    		bool markerCheck = 0;

    		// check if the current barcode exits in current string
    		for(int i=0;i<barcodes.size();i++){
    			if(barcode == barcodes[i]){
    				markerCheck = 1;
    			}
    		}

    		if(markerCheck){
    			continue;
    		}
    		qr.marker = barcode;
    		qr.detect = 1;
    		barcodes.push_back(barcode);
    		markerPub.publish(qr);

         }

    }

}

void MARKER::subscriber(){
//	ros::NodeHandle nh_;

//	ros::Subscriber imageSub = nh_.subscribe("/usb_cam/image_raw", 10, &MARKER::imageCallback, this);
//	ros::spin();

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

