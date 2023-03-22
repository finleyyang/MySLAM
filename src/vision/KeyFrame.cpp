/******************************************************************************
*  FILE_NAME  : KeyFrame.cpp
*  AUTHER     : finley
*  DATA       : 23-3-21
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#include "vision/KeyFrame.h"

#include <utility>
namespace my_slam
{
	KeyFrame::~KeyFrame() = default;
	KeyFrame::KeyFrame() = default;

	KeyFrame::KeyFrame(cv::Mat image, cv::Mat imageright, cv::Mat K, cv::Mat D) : Frame(std::move(image),
		std::move(imageright),
		std::move(K),
		std::move(D))
	{

	}
}