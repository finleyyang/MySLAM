//
// Created by finley on 23-3-17.
//
#include "ORBextractor.h"

namespace my_slam {
    ORBextractor::ORBextractor() = default;

    ORBextractor::~ORBextractor() = default;


    void ORBextractor::operator()(const cv::_InputArray &image,
                                  std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) {
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
        detector->detect(image, keypoints);
        descriptor->compute(image, keypoints, descriptors);
    }
}
