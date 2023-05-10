/******************************************************************************
*  FILE_NAME  : CeresStruct.h
*  AUTHER     : finley
*  DATA       : 23-4-3
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_INCLUDE_OPTIM_CERESSTRUCT_H_
#define MYSLAM_INCLUDE_OPTIM_CERESSTRUCT_H_
#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function.h>
#include <opencv2/core/types.hpp>
#pragma once
#include <utility>
#include <vector>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
namespace my_slam
{
	class ReprojectionErrorPosOptim
	{
	 public:
		ReprojectionErrorPosOptim(Eigen::Vector3d p3s, cv::Point2f p2s, cv::Mat K)
			: _p3s(p3s), _p2s(p2s), _K(K)
		{
		}

		template<typename T>
		bool operator()(const T* const cere_rot,
			const T* const cere_tranf,
			T* residual) const
		{
			T pp[3];
			pp[0] = T(_p3s.x());
			pp[1] = T(_p3s.y());
			pp[2] = T(_p3s.z());

			T p[3];

			ceres::AngleAxisRotatePoint(cere_rot, pp, p);

			p[0] += cere_tranf[0];
			p[1] += cere_tranf[1];
			p[2] += cere_tranf[2];

			T xp = p[0] / p[2];
			T yp = p[0] / p[2];

			T up = xp * T(_K.at<float>(0, 0)) + T(_K.at<float>(0, 2));
			T vp = yp * T(_K.at<float>(1, 1)) + T(_K.at<float>(1, 2));

			residual[0] = T(_p2s.x) - up;
			residual[1] = T(_p2s.y) - vp;
			return true;
		}

		static ceres::CostFunction* Create(Eigen::Vector3d p3, cv::Point2f p2, cv::Mat K)
		{
			return (new ceres::AutoDiffCostFunction<ReprojectionErrorPosOptim,
			                                        2,
			                                        3,
			                                        3>(new ReprojectionErrorPosOptim(p3, p2, K)));
		}

	 private:
		Eigen::Vector3d _p3s;
		cv::Point2f _p2s;
		cv::Mat _K;
	};
}

#endif //MYSLAM_INCLUDE_OPTIM_CERESSTRUCT_H_
