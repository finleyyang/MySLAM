/******************************************************************************
*  FILE_NAME  : LocalMap.h
*  AUTHER     : finley
*  DATA       : 23-3-29
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_SRC_VISION_LOCALMAP_H_
#define MYSLAM_SRC_VISION_LOCALMAP_H_

#pragma once
#include "vision/Map.h"
#include "vision/KeyFrame.h"
namespace my_slam
{
	class KeyFrame;
	class Map;

	class LocalMap
	{
	 public:
		LocalMap(Map* pmp);
		~LocalMap();

		void InsertKeyFrame(KeyFrame* pKF);

	 protected:

		std::list<KeyFrame*> ml_NewKeyFrames;

		KeyFrame* mp_currentKeyFrame;
		Map* mp_Map;

		bool mb_AbortBA;
	};
}

#endif //MYSLAM_SRC_VISION_LOCALMAP_H_
