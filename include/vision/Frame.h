//
// Created by finley on 23-3-17.
//

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H
#pragma one
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <spdlog/spdlog.h>

#include "vision/ORBextractor.h"
#include "vision/ORBmatcher.h"
#include "vision/KeyFrame.h"

namespace my_slam
{

	class MapPoint;
	class KeyFrame;

	class Frame
	{
	 public:
		Frame();

		~Frame();

		Frame(Frame const &F);

		Frame(const cv::Mat& image, const cv::Mat& imageright, const cv::Mat& K, const cv::Mat& D, const float& b);

		void ExtractORB();

		void StereoMatch();

		Eigen::Vector3d UnprojectStereo(const int &i);

		void SetPose(Eigen::Matrix4d Tcw);

		void UpdatePose();

		// using for test
		void ShowORB() const;

		void showstereomatch();

	 public:
		cv::Mat m_image, m_imageRight;

		int m_rows{}, m_cols{};

		int N{};

		ORBExtractor* ORBextractor{}, * ORBextractorRight{};

		std::vector<cv::KeyPoint> mv_keypoints;
		std::vector<cv::KeyPoint> mv_keypointsRight;

		//对应右目的像素坐标
		std::vector<float> mv_uRight;
		std::vector<float> mv_Depth;

		cv::Mat m_descriptors, m_descriptorsRight;

		std::vector<MapPoint*> mvp_mapPoints;

		std::vector<bool> mvb_Outlier;

		float m_b;
		//z = bf / d;
		//m_bf = baseline * length_focal
		float m_bf{386.1448};

		cv::Mat m_K;
		cv::Mat m_D;

		float fx, fy, cx, cy, invfx, invfy;

		//*cw,世界坐标转相机坐标
		Eigen::Matrix4d m_Tcw;

		int mi_FId;

		KeyFrame* mp_referenceKeyFrame;

	 protected:
		Eigen::Matrix3d m_Rcw;
		Eigen::Vector3d m_tcw;
		Eigen::Matrix3d m_Rwc;
		Eigen::Vector3d m_Ow;
	};
}

#endif //MYSLAM_FRAME_H
