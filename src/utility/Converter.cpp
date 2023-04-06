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

	Eigen::Matrix4d Converter::toMatrix4d(const double R[3], const double t[3])
	{
		Eigen::Vector3d R_e;
		Eigen::Vector3d t_e;
		for(int i = 0; i< 3; i++)
		{
			R_e[i] = R[i];
			t_e[i] = t[i];
		}
		Eigen::Matrix4d mat4d = Eigen::Matrix4d::Identity();
		Eigen::Matrix3d rot3d;
		auto n2 = R_e.norm();
		if(n2<std::numeric_limits<double>::epsilon())
			rot3d = Eigen::AngleAxisd(n2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
		else
			rot3d = Eigen::AngleAxisd (n2, R_e.normalized()).toRotationMatrix();

		mat4d.block<3, 3>(0, 0) = rot3d;
		mat4d.block<3, 1>(0, 3) = t_e;
		return mat4d;
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