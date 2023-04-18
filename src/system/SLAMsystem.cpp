/******************************************************************************
*  FILE_NAME  : SLAMsystem.cpp
*  AUTHER     : finley
*  DATA       : 23-3-30
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "system/SLAMsystem.h"
#include "view/Draw.h"
#include "view/FrameDraw.h"
#include "vision/LocalMap.h"
#include "vision/ORBvocabulary.h"
#include <cstdlib>
#include <spdlog/spdlog.h>

#include <ctime>

namespace my_slam
{
	SLAMsystem::SLAMsystem() = default;

	SLAMsystem::~ SLAMsystem() = default;

	SLAMsystem::SLAMsystem(const std::string& strVocFile,
		const std::string& strSettingsFile,
		const SLAMsystem::e_Sensor sensor,
		const bool bUseViewer) : m_Sensor(sensor)
	{
		spdlog::info("SLAMsystem: START SLAM");

		switch (sensor)
		{
		case e_Sensor::MONOCULAR:
			spdlog::info("SLAMsystem: The type sensor is {}", "MONOCULAR");
			break;
		case e_Sensor::STEREO:
			spdlog::info("SLAMsystem: The type sensor is {}", "STEREO");
			break;
		case e_Sensor::RGBD:
			spdlog::info("SLAMsystem: The type sensor is {}", "RGBD");
			break;
		case e_Sensor::IMU:
			spdlog::info("SLAMsystem: The type sensor is {}", "IMU");
			break;
		}

		mp_ORBvocabulary = new ORBvocabulary();

		spdlog::debug("SLAMsystem: Load the ORB vocabulary");
		bool bvocLoad = mp_ORBvocabulary->loadFromTextFile(strVocFile);
		if (!bvocLoad)
		{
			spdlog::error("SLAMsystem: Falied to open the vocabulary");
			exit(-1);
		}
		spdlog::debug("SLAMsystem: Load the ORB vocabulary finished");

		mp_Map = new Map();

		mp_Draw = new Draw(mp_Map);

		mp_frameDraw = new FrameDraw(mp_Map);

		mp_Tracker = new Tracker(mp_Map, mp_Draw, mp_frameDraw, mp_ORBvocabulary);
		mp_Tracker->LoadParam(strSettingsFile);

		mp_localMap = new LocalMap(mp_Map);
		mp_Tracker->SetLocalMap(mp_localMap);

	}

	void SLAMsystem::TrackStereoRGB(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timestamp)
	{
		if (m_Sensor != e_Sensor::STEREO)
		{
			spdlog::error("The sensor is not STEREO, but call the function of STEREO");
		}

		Eigen::Matrix4d Tcw = mp_Tracker->LoadStereoRGB(imLeft, imRight, timestamp);

	}
}
