/******************************************************************************
*  FILE_NAME  : testStereoMatcher.cpp
*  AUTHER     : finley
*  DATA       : 23-3-22
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>
#include <gtest/gtest.h>

#include "system/SLAMsystem.h"

TEST(MyslamTest, StereoMatcher)
{
	spdlog::info("test_StereoMatcher");
	cv::Mat image, imageright;
	cv::Mat K = (cv::Mat_<float>(3, 3) << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1);
	cv::Mat D = (cv::Mat_<float>(5, 1) << 0, 0, 0, 0, 0);
	float fx = K.at<float>(0, 0);
	float fy = K.at<float>(1, 1);
	float cx = K.at<float>(0, 2);
	float cy = K.at<float>(1, 2);
	image = cv::imread("/home/finley/CODE/MySLAM/data/image_00/data/0000000000.png", cv::IMREAD_GRAYSCALE);
	imageright = cv::imread("/home/finley/CODE/MySLAM/data/image_01/data/0000000000.png", cv::IMREAD_GRAYSCALE);
	//std::cout<<image.size()<<std::endl;
	//std::cout<<imageright.size()<<std::endl;
	float b = 0.537;
	my_slam::ORBvocabulary* pvoc;
	my_slam::ORBExtractor* mp_ORBextractorLeft = new my_slam::ORBExtractor(2000, 1.2, 8, 20, 8);
	my_slam::ORBExtractor* mp_ORBextractorRight = new my_slam::ORBExtractor(2000, 1.2, 8, 20, 8);
	my_slam::Frame frame(image, imageright, K, D, b, mp_ORBextractorLeft, mp_ORBextractorRight, pvoc);
	frame.showstereomatch();
	std::vector<cv::KeyPoint> keypointleft, keypointright;
	cv::KeyPoint kpl, kpr;
	cv::Mat imagergb, imagerightrgb;
	cv::cvtColor(image, imagergb, cv::COLOR_GRAY2RGB);
	cv::cvtColor(imageright, imagerightrgb, cv::COLOR_GRAY2RGB);
	for (int i = 0; i < frame.N; i++)
	{
		if (frame.mv_uRight[i] != -1)
		{
			kpl = frame.mv_keypoints[i];
			kpr.pt.y = kpl.pt.y;
			kpr.pt.x = frame.mv_uRight[i];
			cv::circle(imagergb, kpl.pt, 1, { 255, 255, 0 }, 1);
			cv::circle(imagerightrgb,
				kpr.pt,
				1,
				{ 255, 255, 0 },
				1);
		}
	}
	cv::imshow("left", imagergb);
	cv::imshow("right", imagerightrgb);
	cv::waitKey(0);
}

int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	::testing::FLAGS_gtest_filter = "MyslamTest.StereoMatcher";
	spdlog::set_level(spdlog::level::debug);
	return RUN_ALL_TESTS();
}