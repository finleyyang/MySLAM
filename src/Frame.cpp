//
// Created by finley on 23-3-17.
//
#include "Frame.h"

#include <utility>
namespace my_slam {
    Frame::Frame() = default;

    Frame::~Frame() = default;

    Frame::Frame(cv::Mat image_, cv::Mat imageright_, cv::Mat K_, cv::Mat D_) : m_image(image_.clone()),
                                                                                m_imageRight(imageright_.clone()),
                                                                                m_K(K_.clone()), m_D(D_.clone()) {
    }

    void Frame::ExtractORB() {
        (*ORBextractor)(m_image, m_keypoints, m_descriptors);
        (*ORBextractorRight)(m_imageRight, m_keypointsRight, m_descriptorsRight);
    }

	void Frame::ShowORB() const
	{
		cv::Mat outimage;
		cv::drawKeypoints(m_image, m_keypoints, outimage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
		cv::Mat outimageRight;
		cv::drawKeypoints(m_imageRight, m_keypointsRight, outimageRight, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
		cv::imshow("left", outimage);
		cv::waitKey(0);
	}
}