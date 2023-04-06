/******************************************************************************
*  FILE_NAME  : Tracker.cpp
*  AUTHER     : finley
*  DATA       : 23-3-23
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "system/Tracker.h"
#include "Eigen/src/Core/Matrix.h"
#include "optim/VisionOptimizer.h"
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
		if (m_State == e_TrackingState::NO_IMAGES_YET)
		{
			m_State = e_TrackingState::NOT_INITIALIZED;
		}

		m_lastProcessedState = m_State;

		if(m_State == e_TrackingState::NOT_INITIALIZED)
		{
			StereoInitial();

			if(m_State!=e_TrackingState::OK)
				return;

			mp_frameDraw->Update(this);
		}
		else
		{
			bool b_OK;
			if (m_State == e_TrackingState::OK)
			{
				CheckReplacedPointInLastFrame();

				if (m_Velocity == Eigen::Matrix4d::Zero() || m_currentFrame.mi_FId < mi_LastRelocFrameId + 2)
				{
					b_OK = TrackReferenceKeyFrame();
				}
				else
				{
					b_OK = TrackWithMotionModel();
					if (!b_OK)
						b_OK = TrackReferenceKeyFrame();
				}
			}
			else
			{
				spdlog::warn("Tracker: Tracking: Tracking Lost!!! Relocal");
				b_OK = Relocalization();
			}

			m_currentFrame.mp_referenceKeyFrame = mp_referenceKeyFrame;

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
			spdlog::info("Tracker: StereoInitial: The Map is created with {:04.2f} points", mp_map->NumMapPointsinMap());

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
		spdlog::info("Tracker: TrackReferenceKeyFrame: Track by KeyFrame");
		m_currentFrame.ComputeBoW();

		ORBMatcher matcher(0.7, true);

		std::vector<MapPoint*> vpMapPointsMatches;

		int nmatches = matcher.SearchByBoW(mp_referenceKeyFrame, &m_currentFrame, vpMapPointsMatches);

		if(nmatches<15)
		{
			spdlog::warn("Tracker: TrackReferenceKeyFrame: Track {0:d}th frame by BoW fail, the reference KeyFrame is {0:d}", m_currentFrame.mi_FId, mp_referenceKeyFrame->mi_KFId);
			return false;
		}
		m_currentFrame.mvp_mapPoints = vpMapPointsMatches;
		m_currentFrame.SetPose(m_lastFrame.m_Tcw);

		VisionOptimizer::PoseOptimG2O(&m_currentFrame);

		int nmatchesMap = 0;
		for(int i =0; i<m_currentFrame.N; i++)
		{
			if(m_currentFrame.mvp_mapPoints[i])
			{
				//如果对应的这个点为外点
				if(m_currentFrame.mvb_Outlier[i])
				{
					//从当前帧把这个数据删除
					MapPoint* pMP = m_currentFrame.mvp_mapPoints[i];

					m_currentFrame.mvp_mapPoints[i] = static_cast<MapPoint*>(NULL);
					m_currentFrame.mvb_Outlier[i] = false;
					pMP->mb_trackInView = false;
					pMP->mi_lastFrameSeen = m_currentFrame.mi_FId;
					nmatches--;
				}
				else if(m_currentFrame.mvp_mapPoints[i]->Observations()>0)
					nmatchesMap++;
			}
		}
		// 跟踪成功的数目超过10才认为跟踪成功，否则跟踪失败
		if(nmatches > 10)
			spdlog::info("Tracker: TrackReferenceKeyFrame: Track by KeyFrame success");
		else
			spdlog::warn("Tracker: TrackReferenceKeyFrame: Track by KeyFrame fail");
		return nmatchesMap>=10;
	}

	void Tracker::UpdateLastFrame()
	{
		m_currentFrame.SetPose(m_Velocity * m_lastFrame.m_Tcw);
	}

	bool Tracker::TrackWithMotionModel()
	{
		spdlog::info("Tracker: TrackReferenceKeyFrame: Track by MotionModel");
		ORBMatcher matcher(0.9,true);

	}
}