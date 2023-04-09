/******************************************************************************
*  FILE_NAME  : SLAMsystem.h
*  AUTHER     : finley
*  DATA       : 23-3-30
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_INCLUDE_SYSTEM_SLAMSYSTEM_H_
#define MYSLAM_INCLUDE_SYSTEM_SLAMSYSTEM_H_
#include "view/Draw.h"
#include "view/FrameDraw.h"
#include "vision/LocalMap.h"
#include "vision/Map.h"
#include "vision/ORBvocabulary.h"

#include <spdlog/spdlog.h>

#include "system/Tracker.h"

#pragma once

namespace my_slam
{
	class Tracker;
	class Map;
	class LocalMap;
	class Draw;

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

		void LoadParam();

	 public:

		ORBvocabulary* mp_ORBvocabulary;

		Map* mp_Map;

		e_Sensor m_Sensor;

		Tracker* mp_Tracker;

		Draw* mp_Draw;

		FrameDraw* mp_frameDraw;

		LocalMap* mp_localMap;
	};
}

#endif //MYSLAM_INCLUDE_SYSTEM_SLAMSYSTEM_H_
