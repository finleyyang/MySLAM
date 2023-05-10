/******************************************************************************
*  FILE_NAME  : Optimizer.cpp
*  AUTHER     : finley
*  DATA       : 23-4-3
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "optim/VisionOptimizer.h"
#include "Eigen/src/Core/Matrix.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "optim/CeresStruct.h"
#include "utility/Converter.h"
#include "vision/Frame.h"
#include "vision/MapPoint.h"
#include <ceres/cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <cmath>
#include <cstddef>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <vector>

namespace my_slam
{

	int VisionOptimizer::PoseOptimG2O(Frame* pF)
	{
		g2o::SparseOptimizer optimizer;
		g2o::BlockSolver_6_3::LinearSolverType* linearSolver;
		linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
		g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
		optimizer.setAlgorithm(solver);

		int nInitialCorrespondences = 0;
		g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pF->m_Tcw));
		vSE3->setId(0);
		vSE3->setFixed(false);
		optimizer.addVertex(vSE3);

		const int N = pF->N;

		vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
		vector<size_t> vnIndexEdgeStereo;
		vpEdgesStereo.reserve(N);
		vnIndexEdgeStereo.reserve(N);

		const float deltaStereo = sqrt(7.815);

		for (int i = 0; i < N; ++i)
		{
			MapPoint* pmp = pF->mvp_mapPoints[i];
			if (pmp)
			{
				if (pF->mv_uRight[i] > 0)
				{
					nInitialCorrespondences++;
					pF->mvb_Outlier[i] = false;

					Eigen::Vector3d obs;
					const cv::KeyPoint& kpUn = pF->mv_keypoints[i];
					const float& kp_ur = pF->mv_uRight[i];
					obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

					g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();
					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
					e->setMeasurement(obs);
					const float invSigma2 = pF->mv_invLevelSigma2[kpUn.octave];
					Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
					e->setInformation(Info);

					g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(deltaStereo);

					e->fx = pF->fx;
					e->fy = pF->fy;
					e->cx = pF->cx;
					e->cy = pF->cy;
					e->bf = pF->m_bf;
					Eigen::Vector3d Xw = pmp->GetWorldPose();
					e->Xw[0] = Xw[0];
					e->Xw[1] = Xw[1];
					e->Xw[2] = Xw[2];

					optimizer.addEdge(e);

					vpEdgesStereo.push_back(e);
					vnIndexEdgeStereo.push_back(i);
				}
			}
		}

		if (nInitialCorrespondences < 3)
			return 0;

		int nBad = 0;
		for (size_t it = 0; it < 4; it++)
		{
			vSE3->setEstimate(Converter::toSE3Quat(pF->m_Tcw));

			optimizer.initializeOptimization(0);
			optimizer.optimize(its[it]);

			nBad = 0;

			for (size_t i = 0; i < vpEdgesStereo.size(); i++)
			{
				g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

				const size_t idx = vnIndexEdgeStereo[i];

				if (pF->mvb_Outlier[idx])
				{
					e->computeError();
				}
				const float chi2 = e->chi2();

				if (chi2 > chi2Stereo[it])
				{
					pF->mvb_Outlier[idx] = true;
					e->setLevel(1); // 设置为outlier , level 1 对应为外点,上面的过程中我们设置其为不优化
					nBad++;
				}
				else
				{
					e->setLevel(0); // 设置为inlier , level 0 对应为内点,上面的过程中我们设置其为不优化
					pF->mvb_Outlier[idx] = false;
				}

				if (it == 2)
					e->setRobustKernel(0);
			}
			if (optimizer.edges().size() < 10)
				break;
		}

		g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
		g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
		Eigen::Matrix4d pose = Converter::toMatrix4d(SE3quat_recov);
		pF->SetPose(pose);

		return nInitialCorrespondences - nBad;
	}

	int VisionOptimizer::PoseOpenCV(Frame* pF)
	{
		const int N = pF -> N;
		std::vector<Eigen::Vector3d> p3ds_;
		int nInitialCorrespondences = 0;
		int nBad = 0;
		double crs[3], cts[3];
		std::vector<cv::Point3f> p3ds;
		std::vector<cv::Point2f> p2ds;
		for (size_t it = 0; it < 4; it++)
		{
			nBad = 0;
			nInitialCorrespondences = 0;
			for (int i = 0; i < N; i++)
			{
				MapPoint* pmp = pF->mvp_mapPoints[i];
				if (pmp)
				{
					nInitialCorrespondences++;
					pF->mvb_Outlier[i] = false;
					p3ds_.emplace_back(pmp->GetWorldPose());
					p3ds.emplace_back(cv::Point3f(pmp->GetWorldPose().x(),
						pmp->GetWorldPose().y(),
						pmp->GetWorldPose().z()));
					p2ds.emplace_back(pF->mv_keypoints[i].pt);
				}
			}
			cv::Mat cr, ct;
			cv::solvePnP(p3ds, p2ds, pF->m_K, pF->m_D, cr, ct, cv::SOLVEPNP_ITERATIVE);
			crs[0] = cr.at<double>(0);
			crs[1] = cr.at<double>(1);
			crs[2] = cr.at<double>(2);
			cts[0] = ct.at<double>(0);
			cts[1] = ct.at<double>(1);
			cts[2] = ct.at<double>(2);
			int nBad = 0;
			for (int i = 0; i < N; i++)
			{
				MapPoint* pmp = pF->mvp_mapPoints[i];
				if (pmp)
				{
					cv::Point2f point = pF->Reprojection(i, Converter::toMatrix4d(crs, cts));
					double error =
						sqrt(pow(pF->mv_keypoints[i].pt.x - point.x, 2) + pow(pF->mv_keypoints[i].pt.y - point.y, 2));
					if (error > chi2Stereo[0])
					{
						nBad++;
						pF->mvb_Outlier[i] = true;
					}
				}
			}
		}
		pF->SetPose(Converter::toMatrix4d(crs, cts));
		spdlog::info("Get the {0:d}th frame pose by OpenCV", pF->mi_FId);
		return nInitialCorrespondences - nBad;
	}

	int VisionOptimizer::PoseOptimCeres(Frame* pF)
	{
		const int N = pF->N;

		int nInitialCorrespondences = 0;

		std::vector<Eigen::Vector3d> p3ds;
		std::vector<cv::Point3f> p3ds_;
		std::vector<cv::Point2f> p2ds;

		for (int i = 0; i < N; i++)
		{
			MapPoint* pmp = pF->mvp_mapPoints[i];
			if (pmp)
			{
				nInitialCorrespondences++;
				pF->mvb_Outlier[i] = false;
				p3ds.emplace_back(pmp->GetWorldPose());
				p3ds_.emplace_back(cv::Point3f(pmp->GetWorldPose().x(),
					pmp->GetWorldPose().y(),
					pmp->GetWorldPose().z()));
				p2ds.emplace_back(pF->mv_keypoints[i].pt);
			}
		}
		double ceresRot[3], ceresTrans[3];
		cv::Mat cr, ct;
		double crs[3], cts[3];
		cv::solvePnP(p3ds_, p2ds, pF->m_K, pF->m_D, cr, ct, cv::SOLVEPNP_ITERATIVE);
		crs[0] = cr.at<double>(0);
		crs[1] = cr.at<double>(1);
		crs[2] = cr.at<double>(2);
		cts[0] = ct.at<double>(0);
		cts[1] = ct.at<double>(1);
		cts[2] = ct.at<double>(2);
		int n = 0;
		for (int i = 0; i < p3ds.size(); i++)
		{
			auto pose = Converter::toMatrix4d(crs, cts);
			Eigen::Vector3d
				cameraPointPose = pose.block<3, 3>(0, 0) * p3ds[i] + pose.block<3, 1>(0, 3);
			Eigen::Vector3d
				cameraPointPoseUn
				(cameraPointPose.x() / cameraPointPose.z(), cameraPointPose.y() / cameraPointPose.z(), 1);
			cv::Point2f p2(718.856 * cameraPointPoseUn.x() + 607.1928, 718.856 * cameraPointPoseUn.y() + 186.2157);
			double error = sqrt(pow(p2.x - p2ds[i].x, 2) + pow(p2.y - p2ds[i].y, 2));
			std::cout<<error << std::endl;
			if(error<7.815)
			{
				n++;
				std::cout<<p2ds[i]<< " "<<p2<<std::endl;
			}
		}
		std::cout<<n<<std::endl;
		std::cout<<std::endl;
		int nBad = 0;
		for (int i = 0; i < N; i++)
		{
			MapPoint* pmp = pF->mvp_mapPoints[i];
			if (pmp)
			{
				cv::Point2f point = pF -> Reprojection(i, Converter::toMatrix4d(crs, cts));
				std::cout<<point << std::endl;
				std::cout<<pF->mv_keypoints[i].pt<<std::endl;
				double error = pow(pF->mv_keypoints[i].pt.x - point.x, 2) + pow(pF->mv_keypoints[i].pt.y - point.y, 2);
				std::cout<<error<<std::endl;
				if(error > chi2Stereo[0])
				{
					nBad++;
					pF->mvb_Outlier[i] = true;
				}
			}
		}

		pF->SetPose(Converter::toMatrix4d(ceresRot, ceresTrans));
		spdlog::info("Get the {0:d}th frame pose by Ceres, the pose is", pF->mi_FId);
		std::cout<<Converter::toMatrix4d(ceresRot, ceresTrans)<<std::endl;
		std::cout<<nInitialCorrespondences - nBad<<std::endl;
		return nInitialCorrespondences - nBad;
	}
}