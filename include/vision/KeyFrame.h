/******************************************************************************
*  FILE_NAME  : KeyFrame.h
*  AUTHER     : finley
*  DATA       : 23-3-21
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_SRC_KEYFRAME_H_
#define MYSLAM_SRC_KEYFRAME_H_
#pragma one

#include "vision/Frame.h"
#include "vision/Map.h"


namespace my_slam
{
	class Map;
	class MapPoint;
	class Frame;

	class KeyFrame
	{
	 public:

		KeyFrame(Frame& F, Map* p_map);

		~KeyFrame();

		void AddMapPoint(MapPoint* pmp, const size_t& idx);

		bool isBad();

		std::vector<MapPoint*> GetMapPoints();

	 public:

		cv::Mat m_image, m_imageRight;

		Map* mp_map;

		std::vector<MapPoint*> mvp_mapPoints;

		bool mb_Bad;

		std::vector<cv::KeyPoint> mv_keypoints;
		std::vector<cv::KeyPoint> mv_keypointsRight;

		//对应右目的像素坐标
		std::vector<float> mv_uRight;
		std::vector<float> mv_Depth;

		cv::Mat m_K;
		cv::Mat m_D;

		cv::Mat m_descriptors;

		int N{};

		long unsigned int mi_KFId;
		static long unsigned int mi_LastKFId;
		const long unsigned int mi_FrameId;

		//内部结构std::map<WordID, WordValue>
		DBoW2::BowVector m_BowVec;

		//内部实际储存std::map<NodeId, std::vector<unsigned int>>
		DBoW2::FeatureVector m_FeatVec;

	 protected:
		// const只能在函数初始化列表中声明
		const float fx, fy, cx, cy, invfx, invfy;

		//*cw,世界坐标转相机坐标
		Eigen::Matrix4d m_Tcw;

	};
}

#endif //MYSLAM_SRC_KEYFRAME_H_
