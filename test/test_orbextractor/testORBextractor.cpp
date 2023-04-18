//
// Created by finley on 23-3-17.
//

#include "vision/ORBextractor.h"
#include "vision/Frame.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <spdlog/spdlog.h>
#include <ctime>


int main(){
	spdlog::set_level(spdlog::level::debug);


	clock_t start,end;
	start = clock();

	spdlog::info("test_ORBextractor");
	cv::Mat image, imageright;
	cv::Mat K = (cv::Mat_<float>(3,3)<<718.856, 0, 607.1928, 0, 718.856, 186.2157, 0, 0, 1);
	cv::Mat D = (cv::Mat_<float>(5,1)<< 0, 0, 0, 0, 0);
	image = cv::imread("/home/finley/CODE/MySLAM/data/image_00/data/0000000000.png", 0);
	imageright = cv::imread("/home/finley/CODE/MySLAM/data/image_01/data/0000000000.png", 0);
	float b = 386.1448/718.856;
	my_slam::ORBvocabulary* pvoc;
	my_slam::ORBExtractor* mp_ORBextractorLeft = new my_slam::ORBExtractor(2000, 1.2, 8, 20, 8);
	my_slam::ORBExtractor* mp_ORBextractorRight = new my_slam::ORBExtractor(2000, 1.2, 8, 20, 8);

	my_slam::Frame frame(image, imageright, K, D, b, mp_ORBextractorLeft, mp_ORBextractorRight,pvoc);
	//frame.ExtractORB();

	end = clock();
	double time_s = double((end-start))/CLOCKS_PER_SEC * 1000;
	spdlog::debug("ORBextractor cost time is {:03.5f} ms", time_s);

	frame.ShowORB();
}