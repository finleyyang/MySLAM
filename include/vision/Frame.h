//
// Created by finley on 23-3-17.
//

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H
#include <opencv2/core/types.hpp>
#pragma one
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <spdlog/spdlog.h>

#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"

#include "vision/ORBvocabulary.h"
#include "vision/ORBextractor.h"
#include "vision/ORBmatcher.h"
#include "vision/KeyFrame.h"

#include "utility/Converter.h"

namespace my_slam
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

	class MapPoint;
	class KeyFrame;

	class Frame
	{
	 public:
		Frame();

		~Frame();

		Frame(Frame const& F);

		Frame(const cv::Mat& image,
			const cv::Mat& imageright,
			const cv::Mat& K,
			const cv::Mat& D,
			const float& b,
			ORBExtractor* orbExtractorleft,
			ORBExtractor* orbExtractorright,
			ORBvocabulary* pvoc, const float& ThDepth);

		void ExtractORB();

		void StereoMatch();

		Eigen::Vector3d UnprojectStereo(const int& i);

		cv::Point2f Reprojection(int i, Eigen::Matrix4d pose);

		void SetPose(Eigen::Matrix4d Tcw);

		void UpdatePose();

		void ComputeBoW();

		// using for test
		void ShowORB() const;

		void showstereomatch();

		void showinfomation();

		void AssignFeaturesToGrid();

		bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

		vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

	 public:
		cv::Mat m_image, m_imageRight;

		int N{};

		ORBvocabulary* mp_ORBvocabulary;
		ORBExtractor* mp_ORBextractor, * mp_ORBextractorRight;

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
		float m_bf;

		cv::Mat m_K;
		cv::Mat m_D;

		float fx, fy, cx, cy, invfx, invfy;

		//*cw,世界坐标转相机坐标
		Eigen::Matrix4d m_Tcw;

		float mf_ThDepth;


		long unsigned int mi_FId;
		static long unsigned int m_LastFId;

		KeyFrame* mp_referenceKeyFrame;

		//内部结构std::map<WordID, WordValue>
		DBoW2::BowVector m_BowVec;

		//内部实际储存std::map<NodeId, std::vector<unsigned int>>
		DBoW2::FeatureVector m_FeatVec;

		std::vector<std::size_t> m_Grid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

		Eigen::Matrix3d m_Rcw;
		Eigen::Vector3d m_tcw;
		Eigen::Matrix3d m_Rwc;
		Eigen::Vector3d m_Ow;

		static float mf_gridElementWidthInv;
		static float mf_gridElementHeightInv;

		int mi_scaleLevels;
		float mf_scaleFactor;
		float mf_logScaleFactor;
		vector<float> mv_scaleFactors;
		vector<float> mv_invScaleFactors;
		vector<float> mv_levelSigma2;
		vector<float> mv_invLevelSigma2;
	};
}

#endif //MYSLAM_FRAME_H
