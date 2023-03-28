/******************************************************************************
*  FILE_NAME  : KeyFrame.h
*  AUTHER     : finley
*  DATA       : 23-3-21
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_SRC_KEYFRAME_H_
#define MYSLAM_SRC_KEYFRAME_H_
#pragma one
#include "vision/Frame.h"
#include "vision/Map.h"
namespace my_slam
{
	class Map;
	class MapPoint;

	class KeyFrame : public Frame
	{
	 public:
		KeyFrame();

		~KeyFrame();

		KeyFrame(Frame& F, Map* p_map);

		void AddMapPoint(MapPoint* pmp, const size_t &idx);

		bool isBad();

	 public:

		Map* mp_map;

		std::vector<MapPoint*> mvp_mapPoints;

		bool mb_Bad;

		long unsigned int m_KFId;
		static long unsigned int m_LastId;
		static long unsigned int m_FrameId;

	};
}

#endif //MYSLAM_SRC_KEYFRAME_H_
