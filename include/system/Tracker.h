/******************************************************************************
*  FILE_NAME  : Tracker.h
*  AUTHER     : finley
*  DATA       : 23-3-23
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_SRC_VISION_TRACKER_H_
#define MYSLAM_SRC_VISION_TRACKER_H_
#pragma one

#include "vision/Frame.h"
#include "vision/KeyFrame.h"
#include "vision/MapPoint.h"
#include "vision/LocalMap.h"

#include "view/Draw.h"
#include "view/FrameDraw.h"
#include "view/View.h"

namespace my_slam
{
	class Frame;
	class KeyFrame;
	class LocalMap;

	class Draw;
	class FrameDraw;
	class View;

	class Tracker
	{
	 public:
		Tracker();
		~Tracker();

		enum e_TrackingState{
			SYSTEM_NOT_READY=-1,
			NO_IMAGES_YET=0,
			NOT_INITIALIZED=1,
			OK=2,
			LOST=3
		};

		e_TrackingState m_State;
		e_TrackingState m_lastProcessedState;

	 public:
		void LoadParam();

		Eigen::Matrix4d LoadStereoRGB(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp);

		void Tracking();

		void StereoInitial();

		void CheckReplacedPointInLastFrame();

		bool TrackReferenceKeyFrame();

		void UpdateLastFrame();

		bool TrackWithMotionModel();

		bool Relocalization();

	 public:

		cv::Mat K;
		cv::Mat D;

		float m_b;

		Map* mp_map;

		LocalMap* mp_localMap;

		Frame m_currentFrame;

		Frame m_lastFrame;

		cv::Mat m_imgGray;

		KeyFrame* mp_lastKeyFrame;
		KeyFrame* mp_referenceKeyFrame;

		std::vector<KeyFrame*> mvp_localKeyFrames;
		std::vector<MapPoint*> mvp_localMapPoints;

		Draw* mp_draw;
		FrameDraw* mp_frameDraw;
		View* mp_view;

		Frame m_initialFrame;
		std::vector<int> mv_iniMatches;

		unsigned int mi_LastKeyFrameId;

		unsigned int mi_LastRelocFrameId;

		Eigen::Matrix4d m_Velocity;
	};
}

#endif //MYSLAM_SRC_VISION_TRACKER_H_
