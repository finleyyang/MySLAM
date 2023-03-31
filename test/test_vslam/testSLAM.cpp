/******************************************************************************
*  FILE_NAME  : testSLAM.cpp
*  AUTHER     : finley
*  DATA       : 23-3-31
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#include "system/SLAMsystem.h"
#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <string>

int main()
{
	spdlog::set_level(spdlog::level::debug);
	std::string A, B, C;
	bool a = true;
	my_slam::SLAMsystem SLAM(A, B, my_slam::SLAMsystem::e_Sensor::STEREO, true);
	return 0;
}