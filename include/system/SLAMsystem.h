/******************************************************************************
*  FILE_NAME  : SLAMsystem.h
*  AUTHER     : finley
*  DATA       : 23-3-30
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_INCLUDE_SYSTEM_SLAMSYSTEM_H_
#define MYSLAM_INCLUDE_SYSTEM_SLAMSYSTEM_H_
#pragma once
#include "system/Tracker.h"

namespace my_slam
{
	class Tracker;

	class SLAMsystem
	{
	 public:
		SLAMsystem();
		~SLAMsystem();

		enum e_Sensor{
			MONOCULAR=0,
			STEREO=1,
			RGBD=2,
			IMU=3
		};

	};
}

#endif //MYSLAM_INCLUDE_SYSTEM_SLAMSYSTEM_H_
