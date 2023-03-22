//
// Created by finley on 23-3-17.
//

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H
#pragma one
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include "ORBextractor.h"
#include "ORBmatch.h"

namespace my_slam
{

	class MapPoint;

	class Frame
	{
	 public:
		Frame();

		~Frame();

		Frame(cv::Mat image, cv::Mat imageright, cv::Mat K, cv::Mat D);

		void ExtractORB();

		void ShowORB() const;
	 public:
		cv::Mat m_image, m_imageRight;

		ORBExtractor* ORBextractor{}, * ORBextractorRight{};

		ORBmatch*

		std::vector<cv::KeyPoint> m_keypoints;
		std::vector<cv::KeyPoint> m_keypointsRight;

		//对应右目的像素坐标
		std::vector<float> m_uRight;
		std::vector<float> m_vDepth;

		cv::Mat m_descriptors, m_descriptorsRight;

		std::vector<MapPoint*> m_mapPoints;

	 protected:
		cv::Mat m_K;
		cv::Mat m_D;

		//*cw,世界坐标转相机坐标
		Eigen::Matrix4d m_Tcw;
		Eigen::Matrix3d m_Rcw;
		Eigen::Vector3d m_tcw;
		Eigen::Matrix3d m_Rwc;
		Eigen::Vector3d m_Ow;
	};
}

#endif //MYSLAM_FRAME_H
