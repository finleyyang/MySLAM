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

#include "vision/KeyFrame.h"
namespace my_slam
{
	class KeyFrame;

	class LocalMap{
	 public:
		LocalMap();
		~LocalMap();

		void InsertKeyFrame(KeyFrame *pKF);

	 protected:

		std::list<KeyFrame*> ml_NewKeyFrames;

		KeyFrame* mpCurrentKeyFrame;

		bool mb_AbortBA;
	};
}

#endif //MYSLAM_SRC_VISION_LOCALMAP_H_
