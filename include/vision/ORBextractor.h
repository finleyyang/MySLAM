//
// Created by finley on 23-3-17.
//

#ifndef MY_SLAM_ORBEXTRACTOR_H
#define MY_SLAM_ORBEXTRACTOR_H
#pragma one
#include <opencv2/opencv.hpp>

namespace my_slam
{
	class ORBExtractor
	{
	 public:
		ORBExtractor();

		~ORBExtractor();

		void operator()(cv::InputArray image,
			std::vector<cv::KeyPoint>& keypoints,
			cv::Mat& descriptors);
	};
}


#endif //MY_SLAM_ORBEXTRACTOR_H
