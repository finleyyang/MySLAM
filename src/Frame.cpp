//
// Created by finley on 23-3-17.
//
#include "Frame.h"

#include <utility>
namespace my_slam {
    Frame::Frame() = default;

    Frame::~Frame() = default;

    Frame::Frame(cv::Mat image_, cv::Mat imageright_, cv::Mat K_, cv::Mat D_) : image(image_.clone()),
                                                                                imageRight(imageright_.clone()),
                                                                                K(K_.clone()), D(D_.clone()) {
    }

    void Frame::ExtractORB() {
        (*ORBextractor)(image, keypoints, descriptors);
        (*ORBextractorright)(imageRight, keypointsRight, descriptorsRight);
    }
}