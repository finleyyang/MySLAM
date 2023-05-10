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
	long unsigned int KeyFrame::mi_LastKFId = 0;

	KeyFrame::~KeyFrame() = default;

	KeyFrame::KeyFrame(Frame& F, Map* p_map)
		: m_image(F.m_image),
		  m_imageRight(F.m_imageRight),
		  mv_Depth(F.mv_Depth),
		  mv_keypoints(F.mv_keypoints),
		  mv_keypointsRight(F.mv_keypointsRight),
		  mv_uRight(F.mv_uRight),
		  N(F.N),
		  m_K(F.m_K),
		  m_D(F.m_D),
		  fx(F.fx),
		  fy(F.fy),
		  cx(F.cx),
		  cy(F.cy),
		  invfy(F.invfy),
		  invfx(F.invfx),
		  m_descriptors(F.m_descriptors),
		  mp_map(p_map),
		  mvp_mapPoints(F.mvp_mapPoints),
		  m_FeatVec(F.m_FeatVec),
		  m_BowVec(F.m_BowVec),
		  mi_FrameId(F.mi_FId)
	{
		mi_KFId = mi_LastKFId++;
	}

	void KeyFrame::AddMapPoint(MapPoint* pmp, const size_t& idx)
	{
		mvp_mapPoints[idx] = pmp;
	}

	bool KeyFrame::isBad()
	{
		return mb_Bad;
	}
	std::vector<MapPoint*> KeyFrame::GetMapPoints()
	{
		return mvp_mapPoints;
	}
}