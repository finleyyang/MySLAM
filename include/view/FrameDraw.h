/******************************************************************************
*  FILE_NAME  : FrameDraw.h
*  AUTHER     : finley
*  DATA       : 23-3-29
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_INCLUDE_VIEW_FRAMEDRAW_H_
#define MYSLAM_INCLUDE_VIEW_FRAMEDRAW_H_
#pragma once
#include "system/Tracker.h"
namespace my_slam
{
	class Tracker;

	class FrameDraw
	{
	 public:
		FrameDraw() = default;
		~FrameDraw() = default;

		void Update(Tracker* pTracker);

	 public:
		cv::Mat m_img;
		std::vector<cv::KeyPoint> mv_CurrentKeys;
		int N;
		bool mb_onlyTracking;
		std::vector<bool>mvb_VO, mvb_map;

		std::vector<cv::KeyPoint> mv_iniKeys;
		std::vector<int> mv_iniMatches;
		int m_State;
	};
}
#endif //MYSLAM_INCLUDE_VIEW_FRAMEDRAW_H_
