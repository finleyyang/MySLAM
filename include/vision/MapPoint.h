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
#include "vision/KeyFrame.h"
namespace my_slam
{
	class MapPoint
	{
	 public:
		MapPoint();
		~MapPoint();

	 protected:
		Eigen::Vector3d m_worldPose;

		std::map<KeyFrame*, size_t> m_observations;

		KeyFrame* m_refKF;
	};
}

#endif //MYSLAM_SRC_MAPPOINT_H_
