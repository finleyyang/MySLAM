//
// Created by finley on 23-3-17.
//

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H
#pragma one
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include "ORBextractor.h"
#include "ORBmatcher.h"

namespace my_slam
{

	class MapPoint;

	class Frame
	{
	 public:
		Frame();

		~Frame();

		Frame(const cv::Mat& image, const cv::Mat& imageright, const cv::Mat& K, const cv::Mat& D);

		void ExtractORB();

		void StereoMatch();

		void ShowORB() const;
	 public:
		cv::Mat m_image, m_imageRight;

		int m_rows{}, m_cols{};

		int N{};

		ORBExtractor* ORBextractor{}, * ORBextractorRight{};

		std::vector<cv::KeyPoint> m_keypoints;
		std::vector<cv::KeyPoint> m_keypointsRight;

		//对应右目的像素坐标
		std::vector<float> m_uRight;
		std::vector<float> m_Depth;

		cv::Mat m_descriptors, m_descriptorsRight;

		std::vector<MapPoint*> m_mapPoints;

		float m_b{0.0};
		//z = bf / d;
		//m_bf = baseline * length_focal
		float m_bf{0.0};

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
