/******************************************************************************
*  FILE_NAME  : Optimizer.h
*  AUTHER     : finley
*  DATA       : 23-4-3
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_INCLUDE_OPTIM_OPTIMIZER_H_
#define MYSLAM_INCLUDE_OPTIM_OPTIMIZER_H_
#pragma once
#include "vision/Frame.h"

#include "utility/Converter.h"
#include "optim/CeresStruct.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/types/types_seven_dof_expmap.h"

namespace my_slam
{
	class Optimizer
	{
	 public:
		//只优化当前帧的位姿
		int static PoseOptimG2O(Frame* pF);

		int static PoseOptimCeres(Frame* pF);
	};
}

#endif //MYSLAM_INCLUDE_OPTIM_OPTIMIZER_H_
