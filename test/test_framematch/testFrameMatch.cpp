/******************************************************************************
*  FILE_NAME  : testFrameMatch.cpp
*  AUTHER     : finley
*  DATA       : 23-5-8
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "system/SLAMsystem.h"
#include <opencv2/imgcodecs.hpp>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <string>

int main()
{
	spdlog::set_level(spdlog::level::debug);
	spdlog::info("test_framematch");
	std::string A = "/home/finley/CODE/MySLAM/Vocabulary/ORBvoc.txt";
	std::string B = "/home/finley/CODE/MySLAM/data/KITTI.yaml";
	bool a = true;
	my_slam::SLAMsystem SLAM(A, B, my_slam::SLAMsystem::e_Sensor::STEREO, true);
	cv::Mat left = cv::imread("/home/finley/CODE/MySLAM/data/image_00/data/0000000000.png");
	cv::Mat right = cv::imread("/home/finley/CODE/MySLAM/data/image_01/data/0000000000.png");
	cv::Mat left1 = cv::imread("/home/finley/CODE/MySLAM/data/image_00/data/0000000001.png");
	cv::Mat right1 = cv::imread("/home/finley/CODE/MySLAM/data/image_01/data/0000000001.png");
	cv::Mat K = (cv::Mat_<float>(3,3)<<718.856, 0, 607.1928, 0, 718.856, 186.2157, 0, 0, 1);
	cv::Mat D = (cv::Mat_<float>(5,1)<< 0, 0, 0, 0, 0);

	SLAM.TrackStereoRGB(left, right, 0);
	SLAM.TrackStereoRGB(left1, right1, 0);

}