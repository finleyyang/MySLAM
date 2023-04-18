//
// Created by finley on 23-3-17.
//

#ifndef MY_SLAM_ORBEXTRACTOR_H
#define MY_SLAM_ORBEXTRACTOR_H
#include <opencv2/core/types.hpp>
#include <vector>
#pragma one
#include <opencv2/opencv.hpp>

namespace my_slam
{
	enum {HARRIS_SCORE=0, FAST_SCORE=1 };

	class ExtractorNode
	{
	 public:
		ExtractorNode():mb_NoMore(false){}
		void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);


		std::vector<cv::KeyPoint> mv_Keys;
		cv::Point2i UL, UR, BL, BR;
		std::list<ExtractorNode>::iterator lit;

		bool mb_NoMore;
	};
	class ORBExtractor
	{
	 public:
		ORBExtractor(int _nfeatures, float _scaleFactor, int _nlevels, int _iniThFAST, int _minThFAST);

		~ORBExtractor();

		void operator()(cv::InputArray _image, std::vector<cv::KeyPoint>& _keypoints,
			cv::OutputArray _descriptors);

		std::vector<cv::Mat> mv_imagePyramid;


	 protected:

		std::vector<cv::KeyPoint> DistributeQuadTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
			const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

		void ComputeKeyPointsQuadTree(std::vector<std::vector<cv::KeyPoint>> &vvallkeypoints);

		void ComputePyramid(cv::Mat image);

		std::vector<cv::Point> pattern;
		std::vector<int> mv_Max;
		//计算一个半径为16的，圆的近似坐标
		int mi_Features;
		float mf_scaleFactor;
		int mi_Levels;
		int mi_iniThFAST;
		int mi_minThFAST;
		std::vector<int> mv_featuresPerLevel;
		std::vector<float> mv_scaleFactor; //金字塔各层级的缩放系数 这里的值为{1， 1.2， 1.44， 1.728， 2.074， 2.488， 2.986， 3.583}
		std::vector<float> mv_invScaleFactor;    //金字塔各级缩放系数的倒数 这里的值为{1， 0.833， ...}
		std::vector<float> mv_levelSigma2;  //金字塔各层级的缩放系数的平方 这里的值为{1， 1.44，...}
		std::vector<float> mv_invLevelSigma2; //金字塔各层级的缩放系数的平方的倒数 这里的值为{1， 0.689， ...}
	};
}


#endif //MY_SLAM_ORBEXTRACTOR_H
