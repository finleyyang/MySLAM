/******************************************************************************
*  FILE_NAME  : MapPoint.h
*  AUTHER     : finley
*  DATA       : 23-3-21
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_SRC_MAPPOINT_H_
#define MYSLAM_SRC_MAPPOINT_H_
#pragma one

#include "vision/Frame.h"
#include "vision/KeyFrame.h"

namespace my_slam
{
	class KeyFrame;
	class Map;

	class MapPoint
	{
	 public:
		MapPoint();

		~MapPoint();

		MapPoint(Eigen::Vector3d& Pos, KeyFrame* pRefKF, Map* pMap);

		void AddObservation(KeyFrame* pKF, size_t idx);

		void ComputeBestDistinctiveDescriptors();

		int Observations();

		MapPoint* GetReplaced();

		bool isBad();

		Eigen::Vector3d GetWorldPose();

		float mf_trackProjX;

		float mf_trackProjY;

		float mf_trackProjXR;

		bool mb_trackInView;

		float mf_trackViewCos;

		long unsigned int mi_trackReferenceForFrame;

		long unsigned int mi_lastFrameSeen;

	 protected:
		Eigen::Vector3d m_worldPose;

		std::map<KeyFrame*, size_t> m_observations;  //哪个关键帧，中的第几个特征点

		KeyFrame* mp_refKF;

		Map* mp_map;

		int m_Obs;

		cv::Mat m_descriptor;

		MapPoint* mp_Replaced;

		bool mb_Bad;
	};
}

#endif //MYSLAM_SRC_MAPPOINT_H_
