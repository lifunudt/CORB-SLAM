//
// Created by lifu on 6/19/17.
//

#ifndef PROJECT_PUBTOCLIENT_H
#define PROJECT_PUBTOCLIENT_H

#include "ros/ros.h"
#include "Cache.h"

using namespace ORB_SLAM2;

namespace CORBSLAM_SERVER{

    class PubToClient{

    public:
        PubToClient( Cache* pCache);

        void pubNewKFsToClients( std::set<LightKeyFrame> newKFs );
        void pubNewMPsToClients( std::set<LightMapPoint> newMPs );
        void pubUpdatedKFsToClients( std::set<LightKeyFrame> updatedKFs );
        void pubUpdatedMPSToClients( std::set<LightMapPoint> updatedMPs );

    public:

        ORB_SLAM2::Cache * pCacher;

        std::map<int, cv::Mat> transMs;

        ros::Publisher insertKeyFramesPub;
        ros::Publisher insertMapPointsPub;
        ros::Publisher updateKeyFramePosesPub;
        ros::Publisher updateMapPointPosesPub;

    };

}

#endif //PROJECT_PUBTOCLIENT_H
