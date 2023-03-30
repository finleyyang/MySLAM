/******************************************************************************
*  FILE_NAME  : ORBmatch.h
*  AUTHER     : finley
*  DATA       : 23-3-22
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_INCLUDE_ORBMATCH_H_
#define MYSLAM_INCLUDE_ORBMATCH_H_
#pragma one
#include <opencv2/opencv.hpp>
namespace my_slam
{

	class ORBMatcher
	{
	 public:
		ORBMatcher();
		~ORBMatcher();

		static int calculateDescriptorDistance(const cv::Mat &a, const cv::Mat &b);

	 public:
		static const int TH_LOW;
		static const int TH_HIGH;
		static const int HISTO_LENGTH;
	};
}

#endif //MYSLAM_INCLUDE_ORBMATCH_H_
