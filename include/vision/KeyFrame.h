/******************************************************************************
*  FILE_NAME  : KeyFrame.h
*  AUTHER     : finley
*  DATA       : 23-3-21
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_SRC_KEYFRAME_H_
#define MYSLAM_SRC_KEYFRAME_H_
#pragma one
#include "vision/Frame.h"
namespace my_slam
{
	class KeyFrame : public Frame
	{
	 public:
		KeyFrame();

		~KeyFrame();

		KeyFrame(cv::Mat image, cv::Mat imageright, cv::Mat K, cv::Mat D);
	};
}

#endif //MYSLAM_SRC_KEYFRAME_H_
