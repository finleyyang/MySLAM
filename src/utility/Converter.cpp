/******************************************************************************
*  FILE_NAME  : Converter.cpp
*  AUTHER     : finley
*  DATA       : 23-4-1
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#include "utility/Converter.h"

namespace my_slam
{

	std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat& Descriptors)
	{
		std::vector<cv::Mat> vDesc;
		vDesc.reserve(Descriptors.rows);

		for (int j=0;j<Descriptors.rows;j++)
			vDesc.push_back(Descriptors.row(j));

		return vDesc;
	}
}