/******************************************************************************
*  FILE_NAME  : ORBmatch.cpp
*  AUTHER     : finley
*  DATA       : 23-3-22
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "vision/ORBmatcher.h"
#include "DBoW2/FeatureVector.h"
#include "vision/MapPoint.h"
#include <Eigen/src/Core/Matrix.h>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace my_slam
{
	const int ORBMatcher::TH_HIGH = 100;
	const int ORBMatcher::TH_LOW = 50;
	const int ORBMatcher::HISTO_LENGTH = 30;

	ORBMatcher::ORBMatcher() = default;

	ORBMatcher::~ORBMatcher() = default;

	ORBMatcher::ORBMatcher(float nnratio, bool checkOri) : mf_NNration(nnratio), mb_checkOrientation(checkOri)
	{

	}

	int ORBMatcher::CalculateDescriptorDistance(const cv::Mat& a, const cv::Mat& b)
	{
		const int* pa = a.ptr<int32_t>();
		const int* pb = b.ptr<int32_t>();

		long int dist = 0;
		// 8*32=256bit
		for (int i = 0; i < 8; i++, pa++, pb++)
		{
			unsigned int v = *pa ^ *pb;
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

		const DBoW2::FeatureVector& vFeatVecKF = pKF->m_FeatVec;

		int nmatches = 0;

		vector<int> rotHist[HISTO_LENGTH];
		for (int i = 0; i < HISTO_LENGTH; i++)
			rotHist[i].reserve(500);

		const float factor = HISTO_LENGTH / 360.0f;

		auto KFit = vFeatVecKF.begin();
		auto Fit = F->m_FeatVec.begin();
		auto KFend = vFeatVecKF.end();
		auto Fend = F->m_FeatVec.end();

		while (KFit != KFend && Fit != Fend)
		{
			if (KFit->first == Fit->first)
			{
				auto vIndicesKF = KFit->second;
				auto vIndicesF = Fit->second;
				for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++)
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
							if (mb_checkOrientation)
							{
								float rot = kp.angle - F->mv_keypoints[bestIdxF].angle;
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
			else if (KFit->first < Fit->first)
			{
				KFit = vFeatVecKF.lower_bound(Fit->first);
			}
			else
			{
				Fit = F->m_FeatVec.lower_bound(KFit->first);
			}
		}
		//筛选3，根据方向剔除误匹配的点
		if (mb_checkOrientation)
		{
			int ind1 = -1;
			int ind2 = -1;
			int ind3 = -1;

			ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

			for (int i = 0; i < HISTO_LENGTH; i++)
			{
				if (i == ind1 || i == ind2 || i == ind3)
					continue;
				for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
				{
					vpMapPointMatches[rotHist[i][j]] = static_cast<MapPoint*>(NULL);
					nmatches--;
				}
			}
		}
#if 0
		cv::Mat Fimage;
		cv::cvtColor(F->m_image, Fimage, cv::COLOR_GRAY2RGB);
		cv::Mat PFimage;
		cv::cvtColor(pKF->m_image, PFimage, cv::COLOR_GRAY2RGB);
		for (int i = 0; i < vpMapPointMatches.size(); i++)
		{
			if (vpMapPointMatches[i] != NULL)
			{
				std::cout << vpMapPointMatches[i]->m_observations[pKF] << std::endl;
				std::cout<<F->mv_keypoints[i].pt<<std::endl;
				std::cout<<pKF->mv_keypoints[vpMapPointMatches[i]->m_observations[pKF]].pt<<std::endl;
				cv::circle(Fimage, F->mv_keypoints[i].pt, 5, { 255, 255, 0 }, 1);
				cv::circle(PFimage,
					pKF->mv_keypoints[vpMapPointMatches[i]->m_observations[pKF]].pt,
					5,
					{ 255, 255, 0 },
					1);
				cv::imshow("F", Fimage);
				cv::imshow("PF", PFimage);
				cv::waitKey(0);
			}
		}
#endif
		return nmatches;
	}

	int ORBMatcher::SearchByBoW(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint*>& vpMatches12)
	{

	}
	void ORBMatcher::ComputeThreeMaxima(std::vector<int>* histo, const int L, int& ind1, int& ind2, int& ind3)
	{
		int max1 = 0;
		int max2 = 0;
		int max3 = 0;

		for (int i = 0; i < L; i++)
		{
			const int s = histo[i].size();
			if (s > max1)
			{
				max3 = max2;
				max2 = max1;
				max1 = s;
				ind3 = ind2;
				ind2 = ind1;
				ind1 = i;
			}
			else if (s > max2)
			{
				max3 = max2;
				max2 = s;
				ind3 = ind2;
				ind2 = i;
			}
			else if (s > max3)
			{
				max3 = s;
				ind3 = i;
			}
		}

		//如果差距太大，就放弃
		if (max2 < 0.1f * (float)max1)
		{
			ind2 = -1;
			ind3 = -1;
		}
		else if (max3 < 0.1f * (float)max1)
		{
			ind3 = -1;
		}
	}

	int ORBMatcher::SearchByProjection(Frame& CurrentFrame, const Frame& LastFrame, const float th)
	{
		int nmatches = 0;
		vector<int> rotHist[HISTO_LENGTH];
		for(int i=0;i<HISTO_LENGTH;i++)
			rotHist[i].reserve(500);

		const float factor = HISTO_LENGTH/360.0f;

		const Eigen::Matrix3d Rcw = CurrentFrame.m_Tcw.block<3, 3>(0, 0);
		const Eigen::Vector3d tcw = CurrentFrame.m_Tcw.block<3, 1>(0, 3);

		const Eigen::Vector3d twc = -Rcw.transpose()*tcw;

		const Eigen::Matrix3d Rlw = LastFrame.m_Tcw.block<3, 3>(0, 0);
		const Eigen::Vector3d tlw = LastFrame.m_Tcw.block<3, 1>(0, 3);

		const Eigen::Vector3d tlc = Rlw*twc+tlw;

		const bool bForward = tlc[2] > CurrentFrame.m_b;
		const bool bBackward = -tlc[2] > CurrentFrame.m_b;

		for(int i = 0; i < LastFrame.N; i++)
		{
			MapPoint *pmp = LastFrame.mvp_mapPoints[i];
			if(pmp)
			{
				if(!LastFrame.mvb_Outlier[i])
				{
					Eigen::Vector3d x3Dw = pmp -> GetWorldPose();
					Eigen::Vector3d x3Dc = Rcw*x3Dw+tcw;

					const float xc = x3Dc[0];
					const float yc = x3Dc[1];
					const float invzc = 1.0 / x3Dc[2];

					if(invzc < 0)
						continue;

					float u = CurrentFrame.fx * xc * invzc + CurrentFrame.cx;
					float v = CurrentFrame.fx * yc * invzc + CurrentFrame.cy;

					if(u<0 || u>CurrentFrame.m_image.cols)
						continue;
					if(v<0 || v>CurrentFrame.m_image.rows)
						continue;

					int nLastOctave = LastFrame.mv_keypoints[i].octave;

					float radius = th*CurrentFrame.mv_scaleFactors[nLastOctave];

					vector<size_t> vIndices2;

					if(bForward)
						vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave);
					else if(bBackward)
						vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, 0, nLastOctave);
					else
						vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave-1, nLastOctave+1);

					if(vIndices2.empty())
						continue;

					const cv::Mat dMP = pmp->GetDescriptor();

					int bestDist = 256;
					int bestIdx2 = -1;

					for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
					{
						const size_t i2 = *vit;
						if(CurrentFrame.mvp_mapPoints[i2])
							if(CurrentFrame.mvp_mapPoints[i2]->Observations()>0)
								continue;

						if(CurrentFrame.mv_uRight[i2]>0)
						{
							const float ur = u - CurrentFrame.m_bf*invzc;
							const float er = fabs(ur - CurrentFrame.mv_uRight[i2]);
							if(er>radius)
								continue;
						}

						const cv::Mat &d = CurrentFrame.m_descriptors.row(i2);

						const int dist = CalculateDescriptorDistance(dMP,d);

						if(dist<bestDist)
						{
							bestDist=dist;
							bestIdx2=i2;
						}
					}

					if(bestDist<=TH_HIGH)
					{
						CurrentFrame.mvp_mapPoints[bestIdx2]=pmp;
						nmatches++;
						// 计算匹配点旋转角度差所在的直方图
						if(mb_checkOrientation)
						{
							float rot = LastFrame.mv_keypoints[i].angle-CurrentFrame.mv_keypoints[bestIdx2].angle;
							if(rot<0.0)
								rot+=360.0f;
							int bin = round(rot*factor);
							if(bin==HISTO_LENGTH)
								bin=0;
							assert(bin>=0 && bin<HISTO_LENGTH);
							rotHist[bin].push_back(bestIdx2);
						}
					}
				}
			}
		}
		if(mb_checkOrientation)
		{
			int ind1=-1;
			int ind2=-1;
			int ind3=-1;

			ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

			for(int i=0; i<HISTO_LENGTH; i++)
			{
				if(i!=ind1 && i!=ind2 && i!=ind3)
				{
					for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
					{
						CurrentFrame.mvp_mapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
						nmatches--;
					}
				}
			}
		}

		return nmatches;
	}
}