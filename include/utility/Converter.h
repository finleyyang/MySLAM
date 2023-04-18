/******************************************************************************
*  FILE_NAME  : Converter.h
*  AUTHER     : finley
*  DATA       : 23-4-1
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_INCLUDE_UTILTY_CONVERTER_H_
#define MYSLAM_INCLUDE_UTILTY_CONVERTER_H_
#include <opencv2/core.hpp>
#include "Eigen/Eigen"
#include "Eigen/src/Core/Matrix.h"
#include "g2o/types/se3quat.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/types/types_seven_dof_expmap.h"
namespace my_slam
{
	class Converter
	{
	 public:
		static std::vector<cv::Mat> toDescriptorVector(const cv::Mat& Descriptors);
		static g2o::SE3Quat toSE3Quat(const Eigen::Matrix4d& cvT);
		static Eigen::Matrix4d toMatrix4d(const g2o::SE3Quat& SE3);
		static Eigen::Matrix4d toMatrix4d(const double R[3], const double t[3]);
		static cv::Point3f toPoint3f(const Eigen::Vector3d& Point);
	};
}

#endif //MYSLAM_INCLUDE_UTILTY_CONVERTER_H_
