/******************************************************************************
*  FILE_NAME  : Draw.h
*  AUTHER     : finley
*  DATA       : 23-3-28
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_SRC_VISION_DRAW_H_
#define MYSLAM_SRC_VISION_DRAW_H_
#pragma once

#include "vision/Frame.h"
#include "vision/KeyFrame.h"
#include "vision/MapPoint.h"
#include "vision/Map.h"

namespace my_slam
{
	class map;

	class Draw
	{
	 public:
		Draw(Map *pmp);
		~Draw();

		void SetCurrentCameraPose(Eigen::Matrix4d &pose);

	 public:
		Eigen::Matrix4d m_CameraPose;

		Map* mp_Map;
	};
}

#endif //MYSLAM_SRC_VISION_DRAW_H_
