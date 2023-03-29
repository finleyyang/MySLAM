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
		msp_keyFrames.insert(pkf);
		if (pkf->mi_KFId > mi_maxKFid)
			mi_maxKFid = pkf->mi_KFId;
	}

	void Map::AddMapPoint(MapPoint* pmp)
	{
		msp_mapPoints.insert(pmp);
	}

	void Map::EraseKeyFrame(KeyFrame* pkf)
	{
		msp_keyFrames.erase(pkf);
	}

	void Map::EraseMapPoint(MapPoint* pmp)
	{
		msp_mapPoints.erase(pmp);
	}

	long unsigned int Map::NumMapPointsinMap()
	{
		return msp_mapPoints.size();
	}

	long unsigned int Map::NumKeyFrameinMap()
	{
		return msp_keyFrames.size();
	}

	std::vector<KeyFrame*> Map::GetAllKeyFrames()
	{
		return std::vector<KeyFrame*>(msp_keyFrames.begin(),msp_keyFrames.end());
	}

	std::vector<MapPoint*> Map::GetAllMapPoints()
	{
		return std::vector<MapPoint*>(msp_mapPoints.begin(), msp_mapPoints.end());
	}

	std::vector<MapPoint*> Map::GetReferenceMapPoints()
	{
		return mvp_referenceMapPoints;
	}

	void Map::SetReferenceMapPoints(std::vector<MapPoint*> vpmP)
	{
		mvp_referenceMapPoints = vpmP;
	}
}