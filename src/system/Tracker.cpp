/******************************************************************************
*  FILE_NAME  : Tracker.cpp
*  AUTHER     : finley
*  DATA       : 23-3-23
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "system/Tracker.h"

#include <spdlog/spdlog.h>

namespace my_slam
{

	Tracker::Tracker() = default;

	Tracker::~Tracker() = default;

	void Tracker::LoadParam()
	{

	}

	void Tracker::Tracking()
	{
		if (m_State == NO_IMAGES_YET)
		{
			m_State = NOT_INITIALIZED;
		}

		m_lastProcessedState = m_State;

		if(m_State == NOT_INITIALIZED)
		{
			StereoInitial();
		}
	}

	void Tracker::StereoInitial()
	{
		if (m_currentFrame.N > 500)
		{
			m_currentFrame.SetPose(Eigen::Matrix4d::Identity());

			KeyFrame* pKFini = new KeyFrame(m_currentFrame, mp_map);

			mp_map->AddKeyFrame(pKFini);

			for(int i = 0; i < pKFini->N; i++)
			{
				if(pKFini->mv_Depth[i] > 0)
				{
					Eigen::Vector3d x3D = m_currentFrame.UnprojectStereo(i);

					MapPoint* pMapPoint = new MapPoint(x3D, pKFini, mp_map);

					pMapPoint->AddObservation(pKFini, i);

					pKFini->AddMapPoint(pMapPoint, i);

					pMapPoint->ComputeDistinctiveDescriptors();

					mp_map->AddMapPoint(pMapPoint);

					m_currentFrame.mvp_mapPoints[i]= pMapPoint;
				}
			}
			spdlog::info("The Map is created with {:04.2f} points", mp_map->NumMapPointsinMap());

			mp_localMap->InsertKeyFrame(pKFini);

			m_lastFrame = m_currentFrame;
			mi_LastKeyFrameId = m_currentFrame.mi_FId;
			mp_lastKeyFrame = pKFini;

			mvp_localKeyFrames.push_back(pKFini);
			mvp_localMapPoints = mp_map->GetAllMapPoints();
			mp_referenceKeyFrame = pKFini;
			m_currentFrame.mp_referenceKeyFrame = pKFini;

			mp_map->SetReferenceMapPoints(mvp_localMapPoints);
			mp_map->mvp_keyFrameOrigins.push_back(pKFini);

			mp_draw->SetCurrentCameraPose(m_currentFrame.m_Tcw);

			m_State = OK;
		}
	}
}