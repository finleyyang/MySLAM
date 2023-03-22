//
// Created by finley on 23-3-17.
//

#include "ORBextractor.h"
#include "Frame.h"
#include <opencv2/opencv.hpp>
#include <iostream>
int main(){
	cv::Mat image, imageright;
	cv::Mat K = (cv::Mat_<double>(3,3)<<718.856, 0, 607.1928, 0, 718.856, 186.2157, 0, 0, 1);
	cv::Mat D = (cv::Mat_<double>(5,1)<< 0, 0, 0, 0, 0);
	image = cv::imread("/home/finley/CODE/MySLAM/data/image_00/data/0000000000.png");
	imageright = cv::imread("/home/finley/CODE/MySLAM/data/image_01/data/0000000000.png");
	my_slam::Frame frame(image, imageright, K, D);
	frame.ExtractORB();
	frame.ShowORB();
}