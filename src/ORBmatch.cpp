/******************************************************************************
*  FILE_NAME  : ORBmatch.cpp
*  AUTHER     : finley
*  DATA       : 23-3-22
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "ORBmatch.h"

namespace my_slam
{
	ORBMatch::ORBMatch() = default;

	ORBMatch::~ORBMatch() = default;

	int ORBMatch::calculateDescriptorDistance(const cv::Mat& a, const cv::Mat& b)
	{
		const int *pa = a.ptr<int32_t>();
		const int *pb = b.ptr<int32_t>();

		long int dist=0;
		// 8*32=256bit
		for(int i=0; i<8; i++, pa++, pb++)
		{
			unsigned  int v = *pa ^ *pb;
			v = v - ((v >> 1) & 0x55555555);
			v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
			dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
		}

		return int(dist);
	}
}