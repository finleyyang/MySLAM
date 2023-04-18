/******************************************************************************
*  FILE_NAME  : ORBmatch.h
*  AUTHER     : finley
*  DATA       : 23-3-22
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_INCLUDE_ORBMATCH_H_
#define MYSLAM_INCLUDE_ORBMATCH_H_
#pragma one
#include <opencv2/opencv.hpp>

#include "vision/KeyFrame.h"
#include "vision/Frame.h"
#include "vision/MapPoint.h"

namespace my_slam
{
	class KeyFrame;
	class Frame;
	class MapPoint;

	class ORBMatcher
	{
	 public:
		ORBMatcher();
		~ORBMatcher();

		ORBMatcher(float nnratio, bool checkOri = true);

		static int CalculateDescriptorDistance(const cv::Mat& a, const cv::Mat& b);

		int SearchByBoW(KeyFrame* pKF, Frame* F, std::vector<MapPoint*>& vpMapPointMatches);

		int SearchByBoW(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint*>& vpMatches12);

	 public:
		static const int TH_LOW;
		static const int TH_HIGH;
		static const int HISTO_LENGTH;

	 protected:

		void ComputeThreeMaxima(std::vector<int>* histo, const int L, int& ind1, int& ind2, int& ind3);

		float mf_NNration;
		bool mb_checkOrientation;
	};
}

#endif //MYSLAM_INCLUDE_ORBMATCH_H_
