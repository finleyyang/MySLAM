/******************************************************************************
*  FILE_NAME  : LocalMap.cpp
*  AUTHER     : finley
*  DATA       : 23-3-29
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#include "vision/LocalMap.h"

namespace my_slam
{

	LocalMap::LocalMap(Map* pmp) : mp_Map(pmp), mb_AbortBA(false)
	{
	};

	LocalMap::~ LocalMap() = default;

	void LocalMap::InsertKeyFrame(KeyFrame* pKF)
	{
		ml_NewKeyFrames.push_back(pKF);
		mb_AbortBA = true;
	}
}