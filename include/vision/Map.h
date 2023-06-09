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

#include <set>
#include <mutex>

namespace my_slam
{
	class MapPoint;
	class KeyFrame;

	class Map
	{
	 public:
		Map();

		void AddKeyFrame(KeyFrame* pkf);

		void AddMapPoint(MapPoint* pmp);

		void EraseKeyFrame(KeyFrame* pkf);

		void EraseMapPoint(MapPoint* pmp);

		long unsigned int NumMapPointsinMap();

		long unsigned int NumKeyFrameinMap();

		std::vector<KeyFrame*> GetAllKeyFrames();

		std::vector<MapPoint*> GetAllMapPoints();

		std::vector<MapPoint*> GetReferenceMapPoints();

		void SetReferenceMapPoints(std::vector<MapPoint*> vpmP);

	 public:

		std::vector<KeyFrame*> mvp_keyFrameOrigins;

		std::set<MapPoint*> msp_mapPoints;
		std::set<KeyFrame*> msp_keyFrames;

		std::vector<MapPoint*> mvp_referenceMapPoints;

		std::set<int> a;
		std::set<int> test;

		long unsigned int mi_maxKFid;

		std::mutex mMutexMap;
	};

}
#endif //MYSLAM_SRC_MAP_H_
