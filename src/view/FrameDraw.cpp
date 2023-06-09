/******************************************************************************
*  FILE_NAME  : FrameDraw.cpp
*  AUTHER     : finley
*  DATA       : 23-3-29
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "view/FrameDraw.h"

namespace my_slam
{

	FrameDraw::FrameDraw(my_slam::Map* pMap) : mp_Map(pMap)
	{

	}

	FrameDraw::~FrameDraw()
	{

	}

	void FrameDraw::Update(Tracker* pTracker)
	{
		pTracker->m_imgGray.copyTo(m_img);
		mv_CurrentKeys = pTracker->m_currentFrame.mv_keypoints;
		N = mv_CurrentKeys.size();

		mvb_VO = std::vector<bool>(N, false);
		mvb_map = std::vector<bool>(N, false);

		if (pTracker->m_lastProcessedState == Tracker::OK)
		{
			for (int i = 0; i < N; i++)
			{
				MapPoint* pMP = pTracker->m_currentFrame.mvp_mapPoints[i];
				if (pMP)
				{
					if (!pTracker->m_currentFrame.mvb_Outlier[i])
					{
						if (pMP->Observations() > 0)
							mvb_map[i] = true;
						else
							mvb_VO[i] = true;
					}
				}
			}
		}
		m_State = static_cast<int>(pTracker->m_lastProcessedState);
	}
}