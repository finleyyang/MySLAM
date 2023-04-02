/******************************************************************************
*  FILE_NAME  : ORBvocabulary.h
*  AUTHER     : finley
*  DATA       : 23-4-1
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/

#ifndef MYSLAM_INCLUDE_VISION_ORBVOCABULARY_H_
#define MYSLAM_INCLUDE_VISION_ORBVOCABULARY_H_
#pragma once

#include "FORB.h"
#include "TemplatedVocabulary.h"

namespace my_slam
{
	typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBvocabulary;
}
#endif //MYSLAM_INCLUDE_VISION_ORBVOCABULARY_H_
