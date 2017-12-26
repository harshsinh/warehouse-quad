#include "marker.h"

using namespace cv;
using namespace std;
using namespace zbar;
namespace HMDETECTION{
MARKER::MARKER():shelf(1), row(1), col(1), state(DETECTED){
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
}

void MARKER::lineCallback(const hemd::line::ConstPtr& msg){
	double currentTime = ros::Time::now().toSec();//check the current time

	if(msg->mode==2 && state==DETECTED && ((currentTime-timeBegin)>5.0)){
		state = HOVER;
	}
}

void MARKER::subscriber(){
	ros::NodeHandle nh_;

	//ros::Subscriber imageSub = nh_.subscribe("/usb_cam/image_raw", 10, &MARKER::imageCallback, this);

	VideoCapture cap;

	markerPub = nh_.advertise<hemd::markerInfo>("/warehouse_quad/marker",10); //publisher for markers
    followPub = nh_.advertise<std_msgs::Bool>("warehouse_quad/follow_line",10);//publisher to command line following after detection
    markerImgPub = nh_.advertise<sensor_msgs::Image>("warehouse_quad/marker/image",10);
	ros::Subscriber sub = nh_.subscribe("/warehouse_quad/line",10, &MARKER::lineCallback, this);


	bool check=cap.open("http://192.168.42.129:8080/video?x.mjpeg");
	ros::Rate r(84);
	timeBegin = ros::Time::now().toSec();
	while(ros::ok() & check==1){
		Mat tmp;
		cap >> tmp;
		MARKER::videoCap(tmp); //capture video and do marker operations
		r.sleep();
	}

}

void MARKER::videoCap(cv::Mat tmp){

	if(tmp.empty())
		return;

	if(state==HOVER){
		state=DETECTING;
		advertiseFollow(0);
		return;
	}
	else if(state==DETECTED){
		advertiseFollow(1);
		return;
	}
	else if(state==DETECTING){
		timeBegin = ros::Time::now().toSec();
		advertiseFollow(0);
	}



	cv::Mat crop = cropImg(tmp); //crop the image

	cv::Mat frame;
	cvtColor( crop, frame, CV_BGR2GRAY ); //convert to gray scale
	publishMarkerImg(frame);


	//detect the markers at hover state and store them in hoverBarcode
	detectMarker(frame);

	double currentTime = ros::Time::now().toSec();//check the current time

	cout << barcodeHover.size() <<"\t"<<barcodes.size()<<endl;
	if( barcodeHover.size()==2){  //if both barcodes are detected
		for(int i=0;i<barcodeHover.size();i++){
			bool check=false; //check if the marker exist in barcodes string array
			for(int j=0;j<barcodes.size();j++){
				if(barcodeHover[i]==barcodes[j]){
					check=true;
				}
			}
			if(check==false){
				barcodes.push_back(barcodeHover[i]);
				qr.marker = barcodeHover[i];

				qr.col = col;
				qr.shelf = shelf;
				for(int j=0; j<ylocation.size(); j++){
					if(j==i){
						continue;
					}
					if(ylocation[i]>ylocation[j]){
						qr.row = 1;
					}
					else{
						qr.row = 2;
					}
				}
				markerPub.publish(qr);
			}

		}
		state=DETECTED;
		timeBegin = ros::Time::now().toSec();
		advertiseFollow(1);
		barcodeHover.empty();
		ylocation.empty();
		col++;
		if(col%4==0){
			shelf++;
		}
		if(shelf==2){
			col=1;
		}
		return;
	}
	else if((currentTime-timeBegin) > 30 && barcodeHover.size()==1){
		bool check=false; //check if the marker exist in barcodes string array
		for(int j=0;j<barcodes.size();j++){
			if(barcodeHover[0]==barcodes[j]){
				check=true;
			}
		}
		if(check==false){
			barcodes.push_back(barcodeHover[0]);
			qr.marker=barcodeHover[0];
			qr.shelf = shelf;
			qr.col = col;
			if(ylocation[0]>500){
				qr.row =1;
			}
			else{
				qr.row =2;
			}
			markerPub.publish(qr);
		}
		col++;
		if(col%4==0){
			shelf++;
		}
		if(shelf==2){
			col=1;
		}
		advertiseFollow(1);
		state=DETECTED;
		timeBegin = ros::Time::now().toSec();
		barcodeHover.empty();
		ylocation.empty();
		return;
	}
	else if((currentTime-timeBegin) > 30 && barcodeHover.size()==0){
		state=DETECTED;
		timeBegin = ros::Time::now().toSec();
		col++;
		if(col%4==0){
			shelf++;
		}
		if(shelf==2){
			col=1;
		}
		return;
	}
	else if(state==DETECTING){
		advertiseFollow(0);
	}
	else{
		return;
	}
}

cv::Mat MARKER::cropImg(cv::Mat tmp){
    cv::Rect roi;
    roi.x = 724;
    roi.y = 0;
    roi.width = 600;
    roi.height = 1535;

    return tmp(roi);
}

void MARKER::detectMarker(cv::Mat frame){

	zbar::Image zbar_image(frame.cols, frame.rows, "Y800", frame.data,frame.cols * frame.rows);
	scanner_.scan(zbar_image);

	for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol){

		std::string barcode = symbol->get_data();
		bool markerCheck = 0;
		// check if the current barcode exits in current string

		for(int i=0;i<barcodeHover.size();i++){
			if(barcode == barcodeHover[i]){
				markerCheck = 1;
			}
		}
		for(int j=0; j<barcodes.size();j++){
			if(barcode == barcodes[j]){
				markerCheck =1;
			}
		}

		if(!markerCheck){
			barcodeHover.push_back(barcode);
			ylocation.push_back(symbol->get_location_y(0));
		}
	 }
}

void MARKER::publishMarkerImg(cv::Mat frame){

	cv_bridge::CvImage outImg;
	outImg.header.stamp   = ros::Time::now();
	outImg.encoding = sensor_msgs::image_encodings::MONO8; // Image encoding
	outImg.image    = frame; // Your cv::Mat

	markerImgPub.publish(outImg.toImageMsg());

}
void MARKER::advertiseFollow(bool follow){

	std_msgs::Bool msg;

	msg.data = follow;
	followPub.publish(msg);
}

void MARKER::imageCallback(const sensor_msgs::Image::ConstPtr& msg){

}

}

