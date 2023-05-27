/******************************************************************************
*  FILE_NAME  : MapPoint.cpp
*  AUTHER     : finley
*  DATA       : 23-3-21
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "vision/MapPoint.h"
#include "optim/VisionOptimizer.h"

#include <iterator>
#include <utility>
namespace my_slam
{
	long unsigned int MapPoint::mi_lastId = 0;

	MapPoint::MapPoint() = default;

	MapPoint::~MapPoint() = default;

	MapPoint::MapPoint(const Eigen::Vector3d& Pos, KeyFrame* pRefKF, Map* pMap)
		: m_worldPose(std::move(Pos)), mp_refKF(pRefKF), mp_map(pMap)
	{
		mi_Id = mi_lastId++;
	}

	MapPoint::MapPoint(const Eigen::Vector3d& Pos,  Map* pMap, Frame* pFrame , const int &idxF)
		: m_worldPose(std::move(Pos)), mp_map(pMap)
	{

	}

	void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
	{
		if (m_observations.count(pKF))
			return;

		m_observations[pKF] = idx;

		m_Obs++;
	}

	void MapPoint::ComputeBestDistinctiveDescriptors()
	{
		std::vector<cv::Mat> vDescriptors;

		std::map<KeyFrame*, size_t> observations;

		observations = m_observations;

		if (observations.empty())
			return;

		for (std::map<KeyFrame*, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend;
		     mit++)
		{
			KeyFrame* pKF = mit->first;
			if (!pKF->isBad())
				vDescriptors.push_back(pKF->m_descriptors.row(mit->second));
		}

		if (vDescriptors.empty())
			return;

		const size_t N = vDescriptors.size();
		float Distances[N][N];
		for (size_t i = 0; i < N; ++i)
		{
			Distances[i][i] = 0;
			for (size_t j = 0; j < N; ++j)
			{
				int dist = ORBMatcher::CalculateDescriptorDistance(vDescriptors[i], vDescriptors[j]);
				Distances[i][j] = dist;
				Distances[j][i] = dist;
			}
		}
		int BestMedian = INT_MAX;
		int BestIdx = 0;
		for (size_t i = 0; i < N; i++)
		{
			std::vector<int> vDists(Distances[i], Distances[i] + N);
			std::sort(vDists.begin(), vDists.end());
			int median = vDists[0.5 * (N - 1)];

			if (median < BestMedian)
			{
				BestMedian = median;
				BestIdx = i;
			}
		}
		m_descriptor = vDescriptors[BestIdx].clone();
	}
	int MapPoint::Observations()
	{
		return m_Obs;
	}

	MapPoint* MapPoint::GetReplaced()
	{
		return mp_Replaced;
	}

	bool MapPoint::isBad()
	{
		return mb_Bad;
	}

	cv::Mat MapPoint::GetDescriptor()
	{
		return m_descriptor;
	}

	Eigen::Vector3d MapPoint::GetWorldPose()
	{
		return m_worldPose;
	}
}