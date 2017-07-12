//
// Created by lifu on 6/11/17.
//

#ifndef PROJECT_TRANSPOSE_H
#define PROJECT_TRANSPOSE_H

#include "ros/ros.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "LightKeyFrame.h"
#include "LightMapPoint.h"
#include "SerializeObject.h"

namespace ORB_SLAM2{

    class Cache;

    class KeyFrame;

    class MapPoint;

    class LigthKeyFrame;

    class LightMapPoint;

    class KeyFramePose {
    private:
        friend class boost::serialization::access;
        //serialize LightKeyFrame class
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version){
            ar & KFPId;
            ar & KFPose;
            ar & mConnectedKeyFrameWeights;
            ar & mvpMapPoints ;
        };

    public:
        KeyFramePose();

        KeyFramePose( KeyFrame * tKF );

        long unsigned int KFPId;
        cv::Mat KFPose;
        std::map< LightKeyFrame, int> mConnectedKeyFrameWeights;
        std::vector<LightMapPoint> mvpMapPoints;

    };

    class MapPointPose {
    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version){
            ar & MPPId;
            ar & MPPose;
            ar & mObservations;
        };
    public:
        MapPointPose();
        MapPointPose( MapPoint* tMP);

        long unsigned int MPPId;
        cv::Mat MPPose;
        std::map<LightKeyFrame, size_t> mObservations;

    };
}

#endif //PROJECT_TRANSPOSE_H
