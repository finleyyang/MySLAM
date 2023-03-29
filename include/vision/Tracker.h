/******************************************************************************
*  FILE_NAME  : Tracker.h
*  AUTHER     : finley
*  DATA       : 23-3-23
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_SRC_VISION_TRACKER_H_
#define MYSLAM_SRC_VISION_TRACKER_H_
#pragma one

#include "vision/Frame.h"
#include "vision/KeyFrame.h"
#include "vision/MapPoint.h"
#include "vision/LocalMap.h"

namespace my_slam
{
	class Frame;
	class KeyFrame;
	class LocalMap;

	class Tracker
	{
	 public:
		Tracker();
		~Tracker();

		Frame m_currentFrame;

		enum e_TrackingState{
			SYSTEM_NOT_READY=-1,
			NO_IMAGES_YET=0,
			NOT_INITIALIZED=1,
			OK=2,
			LOST=3
		};

	 protected:
		void LoadParam();

		void Tracking();

		void StereoInitial();

	 public:

		Map* mp_map;

		LocalMap* mp_localMap;

		Frame m_lastFrame;

		KeyFrame* mp_lastKeyFrame;

		std::vector<KeyFrame*> mvp_localKeyFrames;
		std::vector<MapPoint*> mvp_localMapPoints;

		unsigned int mi_LastKeyFrameId;

		unsigned int mi_LastRelocFrameId;
	};
}

#endif //MYSLAM_SRC_VISION_TRACKER_H_
