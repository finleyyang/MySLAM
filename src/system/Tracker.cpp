/******************************************************************************
*  FILE_NAME  : Tracker.cpp
*  AUTHER     : finley
*  DATA       : 23-3-23
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "system/Tracker.h"
#include "Eigen/src/Core/Matrix.h"
#include "vision/MapPoint.h"
#include "vision/ORBmatcher.h"

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace my_slam
{

	Tracker::Tracker()
	{
		m_Velocity = Eigen::Matrix4d::Zero();
	};

	Tracker::~Tracker() = default;

	void Tracker::LoadParam()
	{

	}

	Eigen::Matrix4d Tracker::LoadStereoRGB(const cv::Mat& imRectLeft, const cv::Mat& imRectRight, const double& timestamp)
	{
		m_imgGray = imRectLeft;
		cv::Mat imGrayRight = imRectRight;

		if (m_imgGray.channels() == 3)
		{
			cv::cvtColor(m_imgGray, m_imgGray, cv::COLOR_BGR2GRAY);
			cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
		}
		if(m_imgGray.channels() == 4)
		{
			cv::cvtColor(m_imgGray, m_imgGray, cv::COLOR_BGRA2GRAY);
			cv::cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
		}

		m_currentFrame = Frame(m_imgGray, imGrayRight, K, D, m_b);

		Tracking();

		return m_currentFrame.m_Tcw;
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

			if(m_State!=OK)
				return;

			mp_frameDraw->Update(this);
		}
		else
		{
			bool b_OK;
			CheckReplacedPointInLastFrame();

			if(m_Velocity == Eigen::Matrix4d::Zero()||m_currentFrame.mi_FId < mi_LastRelocFrameId + 2)
			{
				b_OK = TrackReferenceKeyFrame();
			}
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

					pMapPoint->ComputeBestDistinctiveDescriptors();

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

	void Tracker::CheckReplacedPointInLastFrame()
	{
		for (int i = 0; i < m_lastFrame.N; i++)
		{
			MapPoint* pmp = m_lastFrame.mvp_mapPoints[i];
			if(pmp)
			{
				MapPoint* pRep = pmp->GetReplaced();
				if(pRep)
					m_lastFrame.mvp_mapPoints[i] = pRep;
			}
		}
	}
	bool Tracker::TrackReferenceKeyFrame()
	{
		m_currentFrame.ComputeBoW();

		ORBMatcher matcher(0.7, true);

		std::vector<MapPoint*> vpMapPointsMatches;

		int nmatches = matcher.SearchByBoW(mp_referenceKeyFrame, &m_currentFrame, vpMapPointsMatches);

		if(nmatches<15)
		{
			spdlog::warn("Track {0:d}th frame by BoW fail, the reference KeyFrame is {0:d}", m_currentFrame.mi_FId, mp_referenceKeyFrame->mi_KFId);
			return false;
		}

	}
}
