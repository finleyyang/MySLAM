/******************************************************************************
*  FILE_NAME  : Map.h
*  AUTHER     : finley
*  DATA       : 23-3-21
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_SRC_MAP_H_
#define MYSLAM_SRC_MAP_H_
#pragma one
#include "vision/KeyFrame.h"
#include "vision/MapPoint.h"

namespace my_slam
{
	class MapPoint;
	class KeyFrame;
	class Map
	{
	 public:
		Map();
		~Map();

		void AddKeyFrame(KeyFrame *pkf);

		void AddMapPoint(MapPoint *pmp);

		void EraseKeyFrame(KeyFrame *pkf);

		void EraseMapPoint(MapPoint *pmp);

	 protected:
		std::set<MapPoint*> mp_mapPoints;
		std::set<KeyFrame*> mp_keyFrames;

		long unsigned int m_maxKFid;
	};
}
#endif //MYSLAM_SRC_MAP_H_
