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

#include "Frame.h"
#include "KeyFrame.h"

namespace my_slam
{
	class Frame;
	class KeyFrame;

	class Tracker
	{
	 public:
		Tracker();
		~Tracker();

	 protected:
		void LoadParam();

		void Tracking();

		void StereoInitial();
	};
}

#endif //MYSLAM_SRC_VISION_TRACKER_H_
