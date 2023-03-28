/******************************************************************************
*  FILE_NAME  : Tracker.cpp
*  AUTHER     : finley
*  DATA       : 23-3-23
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "vision/Tracker.h"

namespace my_slam
{

	Tracker::Tracker() = default;

	Tracker::~Tracker() = default;

	void Tracker::LoadParam()
	{

	}

	void Tracker::Tracking()
	{

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
			//TO_DO
		}
	}
}
