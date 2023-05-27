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
	class Frame;

	class MapPoint
	{
	 public:
		MapPoint();

		~MapPoint();

		MapPoint(const Eigen::Vector3d& Pos, KeyFrame* pRefKF, Map* pMap);
		MapPoint(const Eigen::Vector3d& Pos,  Map* pMap, Frame* pFrame , const int &idxF);

		void AddObservation(KeyFrame* pKF, size_t idx);

		void ComputeBestDistinctiveDescriptors();

		int Observations();

		MapPoint* GetReplaced();

		bool isBad();

		cv::Mat GetDescriptor();

		Eigen::Vector3d GetWorldPose();

		float mf_trackProjX;

		float mf_trackProjY;

		float mf_trackProjXR;

		bool mb_trackInView;

		float mf_trackViewCos;

		long unsigned int mi_trackReferenceForFrame;

		long unsigned int mi_lastFrameSeen;

		std::map<KeyFrame*, size_t> m_observations;

	 protected:
		Eigen::Vector3d m_worldPose;

		long unsigned int mi_Id;
		static long unsigned int mi_lastId;
		long int mi_firstKFid;
		long int mi_firstFrame;

		KeyFrame* mp_refKF;

		Map* mp_map;

		int m_Obs;

		cv::Mat m_descriptor;

		MapPoint* mp_Replaced;

		bool mb_Bad;
	};
}

#endif //MYSLAM_SRC_MAPPOINT_H_
