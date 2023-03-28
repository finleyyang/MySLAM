//
// Created by finley on 23-3-17.
//
#include "vision/Frame.h"

#include <utility>
namespace my_slam
{
	Frame::Frame() = default;

	Frame::~Frame() = default;

	Frame::Frame(const cv::Mat& image_, const cv::Mat& imageright_, const cv::Mat& K_, const cv::Mat& D_) : m_image(
		image_.clone()),
	                                                                                                        m_imageRight(
		                                                                                                        imageright_.clone()),
	                                                                                                        m_K(K_.clone()),
	                                                                                                        m_D(D_.clone())
	{
		m_cols = m_image.cols;
		m_rows = m_image.rows;
		ExtractORB();

		fx = m_K.at<float>(0, 0);
		fy = m_K.at<float>(1, 1);
		cx = m_K.at<float>(0, 2);
		cy = m_K.at<float>(1, 2);

		invfx = 1.0/fx;
		invfy = 1.0/fy;
	}

	void Frame::ExtractORB()
	{
		(*ORBextractor)(m_image, mv_keypoints, m_descriptors);
		(*ORBextractorRight)(m_imageRight, mv_keypointsRight, m_descriptorsRight);
		N = int(mv_keypoints.size());
	}

	void Frame::ShowORB() const
	{
		cv::Mat outimage;
		cv::drawKeypoints(m_image, mv_keypoints, outimage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
		cv::Mat outimageRight;
		cv::drawKeypoints(m_imageRight,
			mv_keypointsRight,
			outimageRight,
			cv::Scalar::all(-1),
			cv::DrawMatchesFlags::DEFAULT);
		cv::imshow("left", outimage);
		cv::waitKey(0);
	}

	void Frame::StereoMatch()
	{
		mv_uRight = std::vector<float>(N, -1.0f);
		mv_Depth = std::vector<float>(N, -1.0f);
		// 二维vector存储每一行的orb特征点的列坐标
		// 第一个vector的序号存储的特征点的行坐标，第二个vector存储的是特征点的序号
		std::vector<std::vector<size_t>> v_RowIndices(m_rows, std::vector<size_t>());
		const int Nr = int(mv_keypointsRight.size());
		for (size_t iR = 0; iR < Nr; iR++)
		{
			const cv::KeyPoint& kp = mv_keypointsRight[iR];
			const float& kpY = kp.pt.y;

			float r = 2.0;
			const int maxr = ceil(kpY + r);
			const int minr = floor(kpY - r);

			for (int yi = minr; yi <= maxr; yi++)
				v_RowIndices[yi].push_back(iR);
		}

		const float minZ = m_b;
		const float minD = 0;
		const float maxD = m_bf / minZ;

		//存储匹配后的特征点（粗匹配）
		std::vector<std::pair<int, int> > v_DistIdx;
		v_DistIdx.reserve(N);

		const int v_thOrbDist = (ORBMatcher::TH_HIGH + ORBMatcher::TH_LOW) / 2;

		for (size_t iL = 0; iL < N; iL++)
		{
			const cv::KeyPoint& kpl = mv_keypoints[iL];
			const float& vL = kpl.pt.y;
			const float& uL = kpl.pt.x;

			const std::vector<size_t>& v_Candidates = v_RowIndices[vL];

			if (v_Candidates.empty())
				continue;

			const cv::Mat& dL = m_descriptors.row(iL);

			//1. 粗匹配
			int v_bestDist = ORBMatcher::TH_HIGH;
			size_t v_bestIdxR = 0;

			const float minU = uL - maxD;
			const float maxU = uL - minD;

			if (maxU < 0)
				continue;

			for (unsigned long iR : v_Candidates)
			{
				const cv::KeyPoint& kpR = mv_keypointsRight[iR];
				const float& uR = kpR.pt.x;

				if (uR < maxU && uR > minU)
				{
					const cv::Mat& dR = m_descriptorsRight.row(iR);
					const int dist = ORBMatcher::calculateDescriptorDistance(dL, dR);

					if (dist < v_bestDist)
					{
						v_bestDist = dist;
						v_bestIdxR = iR;
					}
				}
			}

			//2.精匹配
			if (v_bestDist < v_thOrbDist)
			{
				const float uR0 = mv_keypointsRight[v_bestIdxR].pt.x;
				const int w = 5;
				cv::Mat Il = m_image.rowRange(int(vL - w), int(vL + w + 1)).colRange(int(uL - w), int(uL + w + 1));
				Il.convertTo(Il, CV_32F);
				Il = Il - Il.at<float>(w, w) * cv::Mat::ones(Il.rows, Il.cols, CV_32F);
				int v_bestBlockDist = INT_MAX;
				int v_bestincR = 0;

				//滑动窗口在右图的移动范围为-5 ~ 5；
				const int L = 5;
				std::vector<float> v_Dists;
				v_Dists.resize(2 * L + 1);
				const float iniu = uR0 - L - w;
				const float endu = uR0 + L + w + 1;

				if (iniu < 0 || endu >= float(m_imageRight.cols))
					continue;

				for (int incR = -L; incR <= +L; incR++)
				{
					cv::Mat Ir = m_imageRight.rowRange(int(vL - w), int(vL + w + 1)).colRange(int(uR0) + incR - w,
						int(uR0) + incR + w + 1);
					Ir.convertTo(Ir, CV_32F);
					Ir = Ir - Ir.at<float>(w, w) * cv::Mat::ones(Ir.rows, Ir.cols, CV_32F);
					auto dist = cv::norm(Il, Ir, cv::NORM_L1);

					if (dist < v_bestBlockDist)
					{
						v_bestBlockDist = dist;
						v_bestincR = incR;
					}
					v_Dists[L + incR] = dist;
				}

				if (v_bestincR == -L || v_bestincR == L)
					continue;

				//亚像素插值

				const float dist1 = v_Dists[L + v_bestincR - 1];
				const float dist2 = v_Dists[L + v_bestincR];
				const float dist3 = v_Dists[L + v_bestincR + 1];

				const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

				if (deltaR < -1 || deltaR > 1)
					continue;

				float bestuR = ((float)uR0 + (float)v_bestincR + deltaR);

				float disparity = (uL - bestuR);

				if (disparity >= minD && disparity <= maxD)
				{
					if (disparity <= 0)
					{
						disparity = 0.01;
						bestuR = uL - 0.01;
					}
					mv_Depth[iL] = m_bf / disparity;
					mv_uRight[iL] = bestuR;

					v_DistIdx.push_back(std::pair(v_bestBlockDist, iL));
				}
			}
		}
		sort(v_DistIdx.begin(), v_DistIdx.end());
		const float median = v_DistIdx[v_DistIdx.size() / 2].first;
		const float thDist = 1.5f * 1.4f * median;

		for (size_t i = v_DistIdx.size() - 1; i >= 0; i--)
		{
			if (v_DistIdx[i].first < thDist)
				break;
			else
			{
				mv_uRight[v_DistIdx[i].second] = -1;
				mv_Depth[v_DistIdx[i].second] = -1;
			}
		}
	}

	Eigen::Vector3d Frame::UnprojectStereo(const int& i)
	{
		//计算该特征点的世界三维坐标
		const float z = mv_Depth[i];
		if(z>0)
		{
			const float u = mv_keypoints[i].pt.x;
			const float v = mv_keypoints[i].pt.y;
			const float x = (u-cx)*z*invfx;
			const float y = (v-cy)*z*invfy;
			Eigen::Vector3d x3Dc(x, y, z);
			return m_Rwc*x3Dc+m_Ow;
		}
		else
			return Eigen::Vector3d();
	}

	void Frame::SetPose(Eigen::Matrix4d Tcw)
	{
		m_Tcw = Tcw;
		UpdatePose();
	}

	void Frame::UpdatePose()
	{
		// block <row col>(row, col)
		m_Rcw = m_Tcw.block<3, 3>(0, 0);
		m_tcw = m_Tcw.block<3, 1>(0, 3);
		m_Rwc = m_Rcw.transpose();
		// 0 = Rcw * mOw + tcw
		m_Ow = -m_Rwc * m_tcw;
	}

	void Frame::showstereomatch()
	{
		std::vector<cv::KeyPoint> keypointleft, keypointright;
		cv::KeyPoint kpl, kpr;
		for (int i = 0; i < N; i++)
		{
			if (mv_uRight[i] != -1)
			{
				kpl = mv_keypoints[i];
				kpr.pt.y = kpl.pt.y;
				kpr.pt.x = mv_uRight[i];
				std::cout << kpl.pt << " " << kpr.pt << std::endl;
			}
		}
	}
}