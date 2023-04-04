/******************************************************************************
*  FILE_NAME  : Converter.cpp
*  AUTHER     : finley
*  DATA       : 23-4-1
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#include "utility/Converter.h"
#include "Eigen/src/Core/Matrix.h"
#include <opencv2/core/types.hpp>

namespace my_slam
{

	std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat& Descriptors)
	{
		std::vector<cv::Mat> vDesc;
		vDesc.reserve(Descriptors.rows);

		for (int j=0;j<Descriptors.rows;j++)
			vDesc.push_back(Descriptors.row(j));

		return vDesc;
	}
	g2o::SE3Quat Converter::toSE3Quat(const Eigen::Matrix4d& cvT)
	{
		Eigen::Matrix3d R;
		R = cvT.block<3, 3>(0, 0);
		Eigen::Vector3d t;
		t = cvT.block<3, 1>(0, 3);
		return g2o::SE3Quat(R, t);
	}
	Eigen::Matrix4d Converter::toMatrix4d(const g2o::SE3Quat& SE3)
	{
		return SE3.to_homogeneous_matrix();
	}
	cv::Point3f Converter::toPoint3f(const Eigen::Vector3d& Point)
	{
		return {static_cast<float>(Point.x()), static_cast<float>(Point.y()), static_cast<float>(Point.z())};
	}
}