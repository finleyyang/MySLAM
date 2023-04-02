/******************************************************************************
*  FILE_NAME  : Converter.h
*  AUTHER     : finley
*  DATA       : 23-4-1
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_INCLUDE_UTILTY_CONVERTER_H_
#define MYSLAM_INCLUDE_UTILTY_CONVERTER_H_
#include <opencv2/core.hpp>

namespace my_slam
{
	class Converter
	{
	 public:
		static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);
	};
}

#endif //MYSLAM_INCLUDE_UTILTY_CONVERTER_H_
