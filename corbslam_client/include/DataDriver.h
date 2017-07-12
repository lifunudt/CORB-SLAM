//
// Created by lifu on 2/15/17.
//

#ifndef PROJECT_DATADRIVER_H
#define PROJECT_DATADRIVER_H

#include "System.h"
#include "Cache.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "LightKeyFrame.h"
#include "LightMapPoint.h"

#include <cstdlib>
#include "ros/ros.h"

#include "corbslam_server/corbslam_insert.h"
#include "corbslam_server/corbslam_update.h"
#include "corbslam_server/corbslam_message.h"


namespace ORB_SLAM2{

    class Cache;
    class KeyFrame;
    class MapPoint;
    class LightKeyFrame;
    class LightMapPoint;
    class DataDriver {
    public:

        DataDriver( Cache * pCache);

        void insertNewKeyFramesToServer( std::set<LightKeyFrame> newInsertedKFs );

        void insertNewMapPointsToServer( std::set<LightMapPoint> newInsertedMPs );

        void updateKeyFramePosesToServer( std::set<LightKeyFrame> tUpdateKFs );

        void updateMapPointPosesToServer(  std::set<LightMapPoint> tUpdateMPs );

    private:

        Cache * mpCache;

    };

} //namespace ORB_SLAM


#endif //PROJECT_DATADRIVER_H
