//
// Created by finley on 23-3-17.
//

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H
#pragma one
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include "ORBextractor.h"

namespace my_slam {

    class Frame {
    public:
        Frame();

        ~Frame();

        Frame(cv::Mat image, cv::Mat imageright, cv::Mat K, cv::Mat D);

        void ExtractORB();

    public:
        cv::Mat image, imageRight;

        ORBextractor *ORBextractor, *ORBextractorright;

        std::vector<cv::KeyPoint> keypoints;
        std::vector<cv::KeyPoint> keypointsRight;

        cv::Mat descriptors, descriptorsRight;

        cv::Mat K;
        cv::Mat D;

    private:

    };
}

#endif //MYSLAM_FRAME_H
