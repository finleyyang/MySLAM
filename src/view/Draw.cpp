/******************************************************************************
*  FILE_NAME  : Draw.cpp
*  AUTHER     : finley
*  DATA       : 23-3-28
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#include "view/Draw.h"

namespace my_slam
{
	Draw::Draw()
	{

	}

	Draw::~Draw()
	{

	}
	void Draw::SetCurrentCameraPose(Eigen::Matrix4d &pose)
	{
		m_CameraPose = pose;
	}
}