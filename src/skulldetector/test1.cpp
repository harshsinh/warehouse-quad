#include<bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <opencv2/nonfree/features2d.hpp> //Thanks to Alessandro
#include <opencv2/imgproc/imgproc.hpp>
#include <array>
#include <initializer_list>
#include <numeric>
#include "openCVToQt.hpp"
#include "complex.h"
#include <opencv2/ml/ml.hpp>
using namespace std;
using namespace cv;
// using namespace OCV;
using CP = std::complex<int>;

cv::Mat frame,img_gray,canny_output,drawing,blurred,thresh1,closing,opening,temp,input;
// cv::VideoCapture cap;
int main(int argc, const char* argv[])
{

   cv::Mat trainingData;
   for(size_t i = 1; i != 10; ++i)
      {
        std::stringstream name,out;
        name<<i<<".png";
        out<<i<<"1.png";
        cout<<name.str()<<"*___*"<<out.str()<<endl;
        frame = cv::imread(name.str());



    // const cv::Mat frame = cv::imread(name.str()); //Load as grayscale
    // bool check=cap.open("http://192.168.0.3:8080/video?x.mjpeg");


      // if(!check)
      //     {
      //       std::cout<<"camera not found"<<endl;
      //       return -1;
      //     }
      // while ( cap.isOpened())
      // if (cap.isOpened())
      //   cap >> frame;
      //
      // if (!frame.empty()) {

// resize(frame,frame,(100,100));
        // cvtColor(frame,img_gray,CV_RGB2GRAY);
         cvtColor (frame, img_gray, CV_BGR2HSV);
         inRange (img_gray,  Scalar(20, 80, 100),  Scalar(40, 255, 255), thresh1);
         GaussianBlur (thresh1, blurred,   Size(11, 11), 0, 0);
         //threshold (blurred, thresh1, 127, 255, CV_THRESH_BINARY);
         morphologyEx (thresh1, closing,  MORPH_CLOSE,  getStructuringElement( MORPH_RECT,  Size(2, 2),   Point(-1, -1)));
         morphologyEx (closing, temp/*opening*/,  MORPH_OPEN,  getStructuringElement( MORPH_RECT,  Size(2, 2),   Point(-1, -1)));
         // Canny (opening, temp, 50, 150, 3);
    cv::SiftFeatureDetector detector;
    std::vector<cv::KeyPoint> keypoints;
    detector.detect(temp, keypoints);

    // Add results to image and save.
    cv::Mat output;
    cv::drawKeypoints(frame, keypoints, output);
    cv::imwrite(out.str(), output);
    // output =
      // cv::imwrite("inou.png",temp);
      //}
      cv::imshow(out.str(),output);
      // reshape it to 1 channel and 1 row
      output.reshape(1, 1);
      trainingData.push_back(frame);
    }
    //convert all of the image to floating point values
    trainingData.convertTo(trainingData, CV_32F);
/*******************************************************************/


/*********************************************************************/





cv::imwrite("train.png",trainingData);
    return 0;
}


// template<typename T = uchar, typename U = float>
// void transform_to_svm_training_data(cv::Mat &input)
// {
//    if(input.isContinuous()){
//        input = input.reshape(1, 1);
//        input.convertTo(input, cv::DataType<U>().depth);
//        return;
//    }
//
//    cv::Mat output(1, input.total() * input.channels(),
//                   cv::DataType<U>().depth);
//    auto output_ptr = output.ptr<U>(0);
//    OCV::for_each_channels<T>(input, [&](T a)
//    {
//        *output_ptr = a;
//        ++output_ptr;
//    });
//
//    input = output;
//
// }
