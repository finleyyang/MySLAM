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

#include <spdlog/spdlog.h>

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

		SLAMsystem(const std::string &strVocFile, const std::string &strSettingsFile, const e_Sensor sensor, const bool bUseViewer = true);

		void TrackStereoRGB(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

	 public:

		e_Sensor m_Sensor;

		Tracker* mp_Tracker;
	};
}

#endif //MYSLAM_INCLUDE_SYSTEM_SLAMSYSTEM_H_
