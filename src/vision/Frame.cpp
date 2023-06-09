//
// Created by finley on 23-3-17.
//
#include "vision/Frame.h"
#include "vision/MapPoint.h"

#include <algorithm>
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <utility>
#include <vector>
namespace my_slam
{

	long unsigned int Frame::m_LastFId = 0;

	Frame::Frame() = default;

	Frame::~Frame() = default;

	Frame::Frame(Frame const& F) : m_image(F.m_image), m_imageRight(F.m_imageRight)
	{

	};

	Frame::Frame(const cv::Mat& image_,
		const cv::Mat& imageright_,
		const cv::Mat& K_,
		const cv::Mat& D_,
		const float& b,
		ORBExtractor* orbExtractorleft,
		ORBExtractor* orbExtractorright,
		ORBvocabulary* pvoc, const float& ThDepth) : m_image(image_.clone()),
	                                                 m_imageRight(imageright_.clone()),
	                                                 m_K(K_.clone()),
	                                                 m_D(D_.clone()),
	                                                 m_b(b),
	                                                 mp_ORBextractor(orbExtractorleft),
	                                                 mp_ORBextractorRight(orbExtractorright),
	                                                 mp_ORBvocabulary(pvoc),
	                                                 mf_ThDepth(ThDepth)
	{
		mi_FId = m_LastFId++;
		spdlog::info("Frame: Process {0:d}th frame", mi_FId);

		fx = m_K.at<float>(0, 0);
		fy = m_K.at<float>(1, 1);
		cx = m_K.at<float>(0, 2);
		cy = m_K.at<float>(1, 2);

		invfx = float(1.0) / fx;
		invfy = float(1.0) / fy;

		m_bf = m_b * ((fx + fy) / 2);

		mi_scaleLevels = orbExtractorleft->GetLevels();
		mf_scaleFactor = orbExtractorleft->GetScaleFactor();
		mf_logScaleFactor = log(mf_scaleFactor);
		mv_scaleFactors = orbExtractorleft->GetScaleFactors();
		mv_invScaleFactors = orbExtractorleft->GetInverseScaleFactors();
		mv_levelSigma2 = orbExtractorleft->GetScaleSigmaSquares();
		mv_invLevelSigma2 = orbExtractorleft->GetInverseScaleSigmaSquares();

		ExtractORB();

		mf_gridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(m_image.cols);
		mf_gridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(m_image.rows);

		StereoMatch();

		mvp_mapPoints = std::vector<MapPoint*>(N, static_cast<MapPoint*>(nullptr));
		mvb_Outlier = std::vector<bool>(N, false);

		AssignFeaturesToGrid();
	}

	void Frame::AssignFeaturesToGrid()
	{
		int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
		for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
			for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
				m_Grid[i][j].reserve(nReserve);
		for(int i=0;i<N;i++)
		{
			const cv::KeyPoint &kp = mv_keypoints[i];

			//存储某个特征点所在网格的网格坐标，nGridPosX范围：[0,FRAME_GRID_COLS], nGridPosY范围：[0,FRAME_GRID_ROWS]
			int nGridPosX, nGridPosY;
			// 计算某个特征点所在网格的网格坐标，如果找到特征点所在的网格坐标，记录在nGridPosX,nGridPosY里，返回true，没找到返回false
			if(PosInGrid(kp,nGridPosX,nGridPosY))
				//记录每个网格中的特征点的数量
				m_Grid[nGridPosX][nGridPosY].push_back(i);
		}
	}

	bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
	{

		posX = round(kp.pt.x*mf_gridElementWidthInv);
		posY = round(kp.pt.y*mf_gridElementHeightInv);

		if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
			return false;

		return true;
	}

	void Frame::ExtractORB()
	{
		(*mp_ORBextractor)(m_image, mv_keypoints, m_descriptors);
		(*mp_ORBextractorRight)(m_imageRight, mv_keypointsRight, m_descriptorsRight);
		N = int(mv_keypoints.size());
		spdlog::info("Frame: ExtractORB: there are {0:d} keypoints found", N);
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
		cv::imshow("right", outimageRight);
		cv::waitKey(0);
	}

	void Frame::StereoMatch()
	{

		mv_uRight = std::vector<float>(N, -1.0f);
		mv_Depth = std::vector<float>(N, -1.0f);
		const int v_thOrbDist = (ORBMatcher::TH_HIGH + ORBMatcher::TH_LOW) / 2;
		// 二维vector存储每一行的orb特征点的列坐标
		// 第一个vector的序号存储的特征点的行坐标，第二个vector存储的是特征点的序号
		const int n_rows = mp_ORBextractor->mv_imagePyramid[0].rows;
		std::vector<std::vector<size_t>> v_RowIndices(n_rows, std::vector<size_t>());
		for (int i = 0; i < n_rows; i++)
			v_RowIndices[i].reserve(200);

		const int Nr = int(mv_keypointsRight.size());
		for (size_t iR = 0; iR < Nr; iR++)
		{
			const cv::KeyPoint& kp = mv_keypointsRight[iR];
			const float& kpY = kp.pt.y;

			float r = 2.f * mv_scaleFactors[mv_keypointsRight[iR].octave];
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

		for (size_t iL = 0; iL < N; iL++)
		{
			const cv::KeyPoint& kpl = mv_keypoints[iL];
			const int& levelL = kpl.octave;
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

				if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
					continue;

				if (uR <= maxU && uR >= minU)
				{
					const cv::Mat& dR = m_descriptorsRight.row(iR);
					const int dist = ORBMatcher::CalculateDescriptorDistance(dL, dR);

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
				const float scaleFactor = mv_invScaleFactors[kpl.octave];
				const float scaleduL = round(kpl.pt.x * scaleFactor);
				const float scaledvL = round(kpl.pt.y * scaleFactor);
				const float scaleduR0 = round(uR0 * scaleFactor);
				const int w = 5;
				cv::Mat Il =
					mp_ORBextractor->mv_imagePyramid[kpl.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(
						scaleduL - w,
						scaleduL + w + 1);
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

				if (iniu < 0 || endu >= mp_ORBextractorRight->mv_imagePyramid[kpl.octave].cols)
					continue;

				for (int incR = -L; incR <= +L; incR++)
				{
					cv::Mat Ir = mp_ORBextractorRight->mv_imagePyramid[kpl.octave].rowRange(scaledvL - w,
						scaledvL + w + 1).colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);

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

				float bestuR = mv_scaleFactors[kpl.octave] * ((float)scaleduR0 + (float)v_bestincR + deltaR);

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
		if (z > 0)
		{
			const float u = mv_keypoints[i].pt.x;
			const float v = mv_keypoints[i].pt.y;
			const float x = (u - cx) * z * invfx;
			const float y = (v - cy) * z * invfy;
			Eigen::Vector3d x3Dc(x, y, z);

			return m_Rwc * x3Dc + m_Ow;
		}
		else
			return Eigen::Vector3d();
	}

	cv::Point2f Frame::Reprojection(int i, Eigen::Matrix4d pose)
	{
		// Rcw * x3D + tcw  世界坐标转相机坐标
		Eigen::Vector3d
			cameraPointPose = pose.block<3, 3>(0, 0) * mvp_mapPoints[i]->GetWorldPose() + pose.block<3, 1>(0, 3);
		Eigen::Vector3d
			cameraPointPoseUn(cameraPointPose.x() / cameraPointPose.z(), cameraPointPose.y() / cameraPointPose.z(), 1);
		//相机坐标转图像坐标
		return cv::Point2f(fx * cameraPointPoseUn.x() + cx, fy * cameraPointPoseUn.y() + cy);
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
				std::cout << kpl.pt << " " << kpr.pt << " " << mv_Depth[i] << std::endl;
			}
		}
	}

	void Frame::ComputeBoW()
	{
		if (m_BowVec.empty())
		{
			std::vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(m_descriptors);

			mp_ORBvocabulary->transform(vCurrentDesc, m_BowVec, m_FeatVec, 4);
		}
	}
	void Frame::showinfomation()
	{
		std::cout << "mv_keypoints:size()" << mv_keypoints.size() << std::endl;
		std::cout << "mv_keypointsRight:size()" << mv_keypointsRight.size() << std::endl;
		std::cout << "mvp_mapPoints.size()" << mvp_mapPoints.size() << std::endl;
	}

	vector<size_t> Frame::GetFeaturesInArea(const float& x,
		const float& y,
		const float& r,
		const int minLevel,
		const int maxLevel) const
	{
		vector<size_t> vIndices;
		vIndices.reserve(N);

		const int nMinCellX = max(0, (int)floor((x - r) * mf_gridElementWidthInv));
		if (nMinCellX >= FRAME_GRID_COLS)
			return vIndices;

		const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x + r) * mf_gridElementWidthInv));
		if (nMaxCellX < 0)
			return vIndices;

		const int nMinCellY = max(0, (int)floor((y - r) * mf_gridElementHeightInv));
		if (nMinCellY >= FRAME_GRID_ROWS)
			return vIndices;

		const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y + r) * mf_gridElementHeightInv));
		if(nMaxCellY < 0)
			return vIndices;

		const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

		for(int ix = nMinCellX; ix <= nMinCellX; ix++)
		{
			for(int iy = nMinCellY; iy <= nMaxCellY; iy++)
			{
				const vector<size_t> vCell = m_Grid[ix][iy];
				if(vCell.empty())
					continue;

				for(size_t j=0, jend=vCell.size(); j<jend; j++)
				{
					const cv::KeyPoint &kp = mv_keypoints[vCell[j]];
					if(bCheckLevels)
					{
						if(kp.octave<minLevel)
							continue;
						if(maxLevel>=0)
							if(kp.octave>maxLevel)
								continue;
					}

					const float distx = kp.pt.x-x;
					const float disty = kp.pt.y-y;

					if(fabs(distx)<r && fabs(disty)<r)
						vIndices.push_back(vCell[j]);
				}
			}
		}
		return vIndices;
	}
}