//
// Created by finley on 23-3-17.
//
#include "vision/ORBextractor.h"

namespace my_slam
{
	ORBExtractor::ORBExtractor() = default;

	ORBExtractor::~ORBExtractor() = default;

	void ORBExtractor::operator()(const cv::_InputArray& image,
		std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
	{
		cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
		cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
		detector->detect(image, keypoints);
		descriptor->compute(image, keypoints, descriptors);
	}
}
