/******************************************************************************
*  FILE_NAME  : testStereoMatcher.cpp
*  AUTHER     : finley
*  DATA       : 23-3-22
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include <iostream>
#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>

#include "vision/Frame.h"

int main()
{
	spdlog::set_level(spdlog::level::debug);
	spdlog::info("test_StereoMatcher");
	cv::Mat image, imageright;
	cv::Mat K = (cv::Mat_<double>(3,3)<<718.856, 0, 607.1928, 0, 718.856, 186.2157, 0, 0, 1);
	cv::Mat D = (cv::Mat_<double>(5,1)<< 0, 0, 0, 0, 0);
	image = cv::imread("/home/finley/CODE/MySLAM/data/image_00/data/0000000000.png", cv::IMREAD_GRAYSCALE);
	imageright = cv::imread("/home/finley/CODE/MySLAM/data/image_01/data/0000000000.png", cv::IMREAD_GRAYSCALE);
	//std::cout<<image.size()<<std::endl;
	//std::cout<<imageright.size()<<std::endl;
	float b = 0.537;
	my_slam::Frame frame(image, imageright, K, D, b);
	frame.StereoMatch();
	frame.showstereomatch();
}