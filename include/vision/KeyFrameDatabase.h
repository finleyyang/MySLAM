/******************************************************************************
*  FILE_NAME  : KeyFrameDatabase.h
*  AUTHER     : finley
*  DATA       : 23-5-22
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_INCLUDE_VISION_KEYFRAMEDATABASE_H_
#define MYSLAM_INCLUDE_VISION_KEYFRAMEDATABASE_H_
#pragma one

#include "vision/ORBvocabulary.h"
namespace my_slam
{
	class KeyFrame;
	class Frame;

	class KeyFrameDatabase
	{
	 public:
		KeyFrameDatabase(const ORBvocabulary &voc);

		void add(KeyFrame* pKF);

		void erase(KeyFrame* pKF);

		void clear();

	};
}


#endif //MYSLAM_INCLUDE_VISION_KEYFRAMEDATABASE_H_
