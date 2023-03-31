/******************************************************************************
*  FILE_NAME  : SLAMsystem.cpp
*  AUTHER     : finley
*  DATA       : 23-3-30
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "system/SLAMsystem.h"
#include "view/FrameDraw.h"
#include <spdlog/spdlog.h>

#include <ctime>

namespace my_slam
{
	SLAMsystem::SLAMsystem()=default;

	SLAMsystem::~ SLAMsystem()=default;

	SLAMsystem::SLAMsystem(const std::string& strVocFile,
		const std::string& strSettingsFile,
		const SLAMsystem::e_Sensor sensor,
		const bool bUseViewer) : m_Sensor(sensor)
	{
		spdlog::info("START SLAM");

		switch (sensor)
		{
		case e_Sensor::MONOCULAR:
			spdlog::info("The type sensor is {}", "MONOCULAR");
		case e_Sensor::STEREO:
			spdlog::info("The type sensor is {}", "STEREO");
		case e_Sensor::RGBD:
			spdlog::info("The type sensor is {}", "RGBD");
		case e_Sensor::IMU:
			spdlog::info("The type sensor is {}", "IMU");
		}

		mp_Tracker = new Tracker();
	}

	void SLAMsystem::TrackStereoRGB(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timestamp)
	{
		if(m_Sensor!=e_Sensor::STEREO)
		{
			spdlog::error("The sensor is not STEREO, but call the function of STEREO");
		}

		Eigen::Matrix4d Tcw = mp_Tracker->LoadStereoRGB(imLeft, imRight, timestamp);

	}
}
