/******************************************************************************
*  FILE_NAME  : KeyFrame.cpp
*  AUTHER     : finley
*  DATA       : 23-3-21
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#include "vision/KeyFrame.h"

#include <utility>
namespace my_slam
{
	long unsigned int KeyFrame::m_LastId=0;

	KeyFrame::KeyFrame() = default;
	KeyFrame::~KeyFrame() = default;

	KeyFrame::KeyFrame(Frame& F, Map* p_map): mp_map(p_map)
	{

	}

	void KeyFrame::AddMapPoint(MapPoint* pmp, const size_t& idx)
	{
		mvp_mapPoints[idx] = pmp;
	}

	bool KeyFrame::isBad()
	{
		return mb_Bad;
	}
}