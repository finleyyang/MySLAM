/******************************************************************************
*  FILE_NAME  : Map.cpp
*  AUTHER     : finley
*  DATA       : 23-3-21
*  BRIEF      :
*  Email      : finleyyang@163.com
******************************************************************************/
#include "vision/Map.h"

namespace my_slam
{
	Map::Map() = default;

	Map::~Map() = default;

	void Map::AddKeyFrame(KeyFrame* pkf)
	{
		mp_keyFrames.insert(pkf);
		if (pkf->m_KFId > m_maxKFid)
			m_maxKFid = pkf->m_KFId;
	}

	void Map::AddMapPoint(MapPoint* pmp)
	{
		mp_mapPoints.insert(pmp);
	}

	void Map::EraseKeyFrame(KeyFrame* pkf)
	{
		mp_keyFrames.erase(pkf);
	}

	void Map::EraseMapPoint(MapPoint* pmp)
	{
		mp_mapPoints.erase(pmp);
	}
}