/******************************************************************************
*  FILE_NAME  : testDBoW.cpp
*  AUTHER     : finley
*  DATA       : 23-4-1
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#include <set>
#include "vision/Frame.h"
#include "vision/Map.h"
#include "vision/KeyFrame.h"

int main()
{
	spdlog::set_level(spdlog::level::debug);
	spdlog::info("test_set");
	cv::Mat image, imageright;
	cv::Mat K = (cv::Mat_<float>(3,3)<<718.856, 0, 607.1928, 0, 718.856, 186.2157, 0, 0, 1);
	cv::Mat D = (cv::Mat_<float>(5,1)<< 0, 0, 0, 0, 0);
	image = cv::imread("/home/finley/CODE/MySLAM/data/image_00/data/0000000000.png", cv::IMREAD_GRAYSCALE);
	imageright = cv::imread("/home/finley/CODE/MySLAM/data/image_01/data/0000000000.png", cv::IMREAD_GRAYSCALE);
	//std::cout<<image.size()<<std::endl;
	//std::cout<<imageright.size()<<std::endl;
	float b = 0.537;
	my_slam::Frame frame(image, imageright, K, D, b);
	my_slam::Map* mp_map = new my_slam::Map();
	my_slam::KeyFrame* pKFini = new my_slam::KeyFrame(frame, mp_map);
	//std::set<my_slam::KeyFrame*> a;
	mp_map->AddKeyFrame(pKFini);
	return 0;
}