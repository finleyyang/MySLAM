/******************************************************************************
*  FILE_NAME  : Optimizer.cpp
*  AUTHER     : finley
*  DATA       : 23-4-3
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "optim/VisionOptimizer.h"
#include "Eigen/src/Core/Matrix.h"
#include "optim/CeresStruct.h"
#include "utility/Converter.h"
#include "vision/MapPoint.h"
#include <ceres/cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

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
				nInitialCorrespondences++;
				pF->mvb_Outlier[i] = false;

				Eigen::Vector3d obs;
				const cv::KeyPoint& kpUn = pF->mv_keypoints[i];
				const float& kp_ur = pF->mv_uRight[i];
				obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

				g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();
				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
				e->setMeasurement(obs);
				Eigen::Matrix3d Info = Eigen::Matrix3d::Identity();
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

		if (nInitialCorrespondences < 3)
			return 0;

		const float chi2Stereo[4] = { 7.815, 7.815, 7.815, 7.815 };
		const int its[4] = { 10, 10, 10, 10 };

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

	int VisionOptimizer::PoseOptimCeres(Frame* pF)
	{
		const int N = pF->N;

		int nInitialCorrespondences = 0;

		std::vector<Eigen::Vector3d> p3ds;
		std::vector<cv::Point2f> p2ds;

		for (int i = 0; i < N; i++)
		{
			MapPoint* pmp = pF->mvp_mapPoints[i];
			if (pmp)
			{
				nInitialCorrespondences++;
				pF->mvb_Outlier[i] = false;
				p3ds.emplace_back(pmp->GetWorldPose());
				p2ds.emplace_back(pF->mv_keypoints[i].pt);
			}
		}
		int nBad = 0;
		double ceresRot[3], ceresTrans[3];
		auto poseR = pF -> m_Rcw;
		auto poset = pF -> m_tcw;
		Eigen::AngleAxisd R(poseR);
		ceresTrans[0] = poset[0];
		ceresTrans[1] = poset[1];
		ceresTrans[2] = poset[2];

		ceresRot[0] = R.axis()[0] * R.angle();
		ceresRot[1] = R.axis()[1] * R.angle();
		ceresRot[2] = R.axis()[2] * R.angle();

		ceres::Problem problem;
		for (size_t i = 0; i < p3ds.size(); i++)
		{
			ceres::LossFunction* loss = new ceres::HuberLoss(0.5);
			problem.AddResidualBlock(ReprojectionErrorPosOptim::Create(p3ds[i], p2ds[i], pF->m_K),
				loss,
				ceresRot,
				ceresTrans);
		}

		// set ceres solver
		ceres::Solver::Options options;
		ceres::Solver::Summary summary;
		options.linear_solver_type = ceres::DENSE_SCHUR;
		options.minimizer_progress_to_stdout = true;
		options.max_num_iterations = 30;

		// solve the BA problem
		ceres::Solve(options, &problem, &summary);

		for(int i = 0; i<p3ds.size(); i++)
		{

		}

		pF->SetPose(Converter::toMatrix4d(ceresRot, ceresTrans));
		spdlog::info("Get the {0:d}th frame pose by Ceres, the pose is", pF->mi_FId);
		std::cout<<Converter::toMatrix4d(ceresRot, ceresTrans)<<std::endl;
		return nInitialCorrespondences - nBad;
	}
}