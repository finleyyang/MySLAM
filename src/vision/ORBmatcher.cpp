/******************************************************************************
*  FILE_NAME  : ORBmatch.cpp
*  AUTHER     : finley
*  DATA       : 23-3-22
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "vision/ORBmatcher.h"
#include "FeatureVector.h"
#include "vision/MapPoint.h"
#include <opencv2/core/types.hpp>
#include <vector>

namespace my_slam
{
	const int ORBMatcher::TH_HIGH = 100;
	const int ORBMatcher::TH_LOW = 50;
	const int ORBMatcher::HISTO_LENGTH = 30;

	ORBMatcher::ORBMatcher() = default;

	ORBMatcher::~ORBMatcher() = default;

	ORBMatcher::ORBMatcher(float nnratio, bool checkOri) : mf_NNration(nnratio), mb_CheckOrientation(checkOri)
	{

	}

	int ORBMatcher::CalculateDescriptorDistance(const cv::Mat& a, const cv::Mat& b)
	{
		const int *pa = a.ptr<int32_t>();
		const int *pb = b.ptr<int32_t>();

		long int dist=0;
		// 8*32=256bit
		for(int i=0; i<8; i++, pa++, pb++)
		{
			unsigned  int v = *pa ^ *pb;
			v = v - ((v >> 1) & 0x55555555);
			v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
			dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
		}

		return int(dist);
	}

	int ORBMatcher::SearchByBoW(KeyFrame* pKF, Frame* F, std::vector<MapPoint*>& vpMapPointMatches)
	{
		const std::vector<MapPoint*> vpMapPointsKF = pKF->GetMapPoints();

		vpMapPointMatches = vector<MapPoint*>(F->N, static_cast<MapPoint*>(NULL));

		const DBoW2::FeatureVector &vFeatVecKF = pKF -> m_FeatVec;

		int nmatches = 0;

		vector<int> rotHist[HISTO_LENGTH];
		for(int i = 0; i < HISTO_LENGTH; i++)
			rotHist[i].resize(500);

		const float factor = HISTO_LENGTH/360.0f;

		auto KFit = vFeatVecKF.begin();
		auto Fit = F->m_FeatVec.begin();
		auto KFend = vFeatVecKF.end();
		auto Fend = F->m_FeatVec.end();

		while(KFit != KFend && Fit != Fend)
		{
			if(KFit -> first==Fit -> first)
			{
				auto vIndicesKF = KFit -> second;
				auto vIndicesF = Fit -> second;
				for(size_t iKF = 0; iKF < vIndicesKF.size(); iKF++)
				{
					auto realIdxKF = vIndicesKF[iKF];

					MapPoint* pmp = vpMapPointsKF[realIdxKF];
					if (!pmp)
						continue;
					if (pmp->isBad())
						continue;

					const cv::Mat& dKF = pKF->m_descriptors.row(realIdxKF);

					int bestDist1 = 256;
					int bestIdxF = -1;
					int bestDist2 = 256;
					for (size_t iF = 0; iF < vIndicesF.size(); iF++)
					{
						auto realIdxF = vIndicesF[iF];
						if (vpMapPointMatches[realIdxF])
							continue;

						const cv::Mat& dF = F->m_descriptors.row(realIdxF);

						const int dist = CalculateDescriptorDistance(dKF, dF);

						if (dist < bestDist1)
						{
							bestDist2 = bestDist1;
							bestDist1 = dist;
							bestIdxF = realIdxF;
						}
						else if (dist < bestDist2)
						{
							bestDist2 = dist;
						}
					}

					//筛选1，匹配距离必须要小于设定阈值
					if (bestDist1 <= TH_LOW)
					{
						//筛选2，最佳匹配要比次佳匹配明显摇好，匹配距离越小越好
						if (float(bestDist1) < mf_NNration * float(bestDist2))
						{
							//Frame中第几个特征点，对应地图点，关键帧的地图点
							vpMapPointMatches[bestIdxF] = pmp;

							const cv::KeyPoint& kp = pKF->mv_keypoints[realIdxKF];

							//统计匹配点的旋转直方图
							if (mb_CheckOrientation)
							{
								float rot = kp.angle - F.mv_keypoints[bestIdxF].angle;
								if (rot < 0.0)
									rot += 360.0f;

								int bin = round(rot * factor);
								if (bin == HISTO_LENGTH)
									bin = 0;
								assert(bin >= 0 && bin < HISTO_LENGTH);
								rotHist[bin].push_back(bestIdxF);
							}
							nmatches++;
						}
					}
				}
				KFit++;
				Fit++;
			}
			else if(KFit->first < Fit->first)
			{
				KFit = vFeatVecKF.lower_bound(Fit->first);
			}
			else
			{
				Fit = F.m_FeatVec.lower_bound(KFit->first);
			}
		}
		//筛选3，根据方向剔除误匹配的点
		if(mb_CheckOrientation)
		{
			int ind1 = -1;
			int ind2 = -1;
			int ind3 = -1;

			ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

			for (int i = 0; i < HISTO_LENGTH; i++) {
			    if(i == ind1||i == ind2||i == ind3)
				    continue;
				for(size_t j = 0, jend=rotHist[i].size(); j < jend; j++)
				{
					vpMapPointMatches[rotHist[i][j]] = static_cast<MapPoint*>(NULL);
					nmatches--;
				}
			}
		}
		return nmatches;
	}

	int ORBMatcher::SearchByBoW(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint*>& vpMatches12)
	{

	}
	void ORBMatcher::ComputeThreeMaxima(std::vector<int>* histo, const int L, int& ind1, int& ind2, int& ind3)
	{
		int max1=0;
		int max2=0;
		int max3=0;

		for(int i=0; i<L; i++)
		{
			const int s = histo[i].size();
			if(s>max1)
			{
				max3=max2;
				max2=max1;
				max1=s;
				ind3=ind2;
				ind2=ind1;
				ind1=i;
			}
			else if(s>max2)
			{
				max3=max2;
				max2=s;
				ind3=ind2;
				ind2=i;
			}
			else if(s>max3)
			{
				max3=s;
				ind3=i;
			}
		}

		//如果差距太大，就放弃
		if(max2<0.1f*(float)max1)
		{
			ind2=-1;
			ind3=-1;
		}
		else if(max3<0.1f*(float)max1)
		{
			ind3=-1;
		}
	}
}