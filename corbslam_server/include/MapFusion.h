//
// Created by lifu on 6/6/17.
//

#ifndef PROJECT_MAPFUSION_H
#define PROJECT_MAPFUSION_H

//#include <include/KeyFrame.h>

#include "KeyFrame.h"
#include "ros/ros.h"
#include "corbslam_client/corbslam_update.h"
#include "corbslam_client/corbslam_insert.h"
#include "corbslam_client/corbslam_message.h"
#include "ServerMap.h"
#include "TransPose.h"
#include "ORBmatcher.h"
#include "PnPsolver.h"
#include "MapDrawer.h"
#include "ServerMapView.h"
#include "PubToClient.h"
#include "GlobalOptimize.h"

using namespace ORB_SLAM2;

namespace CORBSLAM_SERVER{

    class MapFusion{

    public:

        MapFusion( std::string strSettingPath );

        void createSubServerMap( int mapId );

        void runPubTopic();

        bool insertKeyFrameToMap(corbslam_client::corbslam_insert::Request &req,
                                 corbslam_client::corbslam_insert::Response &res);

        bool insertMapPointToMap(corbslam_client::corbslam_insert::Request &req,
                                 corbslam_client::corbslam_insert::Response &res);

        bool updateKeyFrameToMap(corbslam_client::corbslam_update::Request &req,
                                 corbslam_client::corbslam_update::Response &res);

        bool updateMapPointToMap(corbslam_client::corbslam_update::Request &req,
                                 corbslam_client::corbslam_update::Response &res);

        void fuseSubMapToMap();

        void resentGlobalMapToClient();

        bool loadORBVocabulary(const string &strVocFile);

        void createKeyFrameDatabase();

        bool mapFuse( ServerMap* sMapx, ServerMap* sMapy);

        bool mapFuseToGlobalMap( ServerMap * sMap );

        bool detectKeyFrameInServerMap( ServerMap * sMap, KeyFrame* tKF, cv::Mat &newPose, std::vector<KeyFrame *> &candidateKFs);

        void insertServerMapToGlobleMap( ServerMap * sMap, cv::Mat To2n);

    private:

        // ORB vocabulary used for place recognition and feature matching.
        ORBVocabulary *mpVocabulary;

        GlobalOptimize * mpGBA;

        std::string mpStrSettingPath;

        MapDrawer * mpGlobalMapDrawer;
        ServerMapView * mpSMView;
        std::thread * mptViewer;
        PubToClient * pubToClient;

        bool ifSubToGlobalMap[100];

        bool ifNullGlobalMap;

        bool resentGlobalMap;

        std::mutex resentGlobalMapMutex;
        std::mutex nullGlobalMapMutex;

        cv::Mat subMapTransM[100];

        // KeyFrame database for place recognition (relocalization and loop detection).
        KeyFrameDatabase *mpKeyFrameDatabase;

        std::map<int, ServerMap *> serverMap;

        ServerMap * globalMap;

        std::mutex mStreamInOutPutMutex;

        std::mutex mSubMapUpdatedMutex;

    };
}


#endif //PROJECT_MAPFUSION_H
