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
	long unsigned int KeyFrame::m_LastId = 0;

	KeyFrame::~KeyFrame() = default;

	KeyFrame::KeyFrame(Frame& F, Map* p_map)
		: mv_Depth(F.mv_Depth), mv_keypoints(F.mv_keypoints), mv_keypointsRight(F.mv_keypointsRight),
		  mv_uRight(F.mv_uRight), m_K(F.m_K), m_D(F.m_D), fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfy(F.invfy),
		  invfx(F.invfx), mp_map(p_map)
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