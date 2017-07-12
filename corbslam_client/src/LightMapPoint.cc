//
// Created by lifu on 17-2-2.
//


#include "System.h"
#include "Cache.h"
#include "LightKeyFrame.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include "LightMapPoint.h"


namespace ORB_SLAM2 {

    LightMapPoint::LightMapPoint() {
        this->mnMapPointId = 0;
        this->mpCache = nullptr;
    }
    LightMapPoint::LightMapPoint(MapPoint *pMP) {
        if( pMP) {
            this->mnMapPointId = pMP->mnId;
            this->mpCache = pMP->getCache();
        }else {
            this->mnMapPointId = 0;
            this->mpCache = nullptr;
        }

    }
    LightMapPoint::LightMapPoint(long unsigned int pId, Cache *pCache) {

        this->mnMapPointId = pId;
        this->mpCache = pCache;

    }
    MapPoint* LightMapPoint::getMapPoint() const {

        if( this->mpCache )
            return this->mpCache->getMapPointById( this->mnMapPointId);
        else
            return nullptr;

    }


}