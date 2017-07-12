//
// Created by lifu on 6/11/17.
//

#include "TransPose.h"

namespace ORB_SLAM2{

    KeyFramePose::KeyFramePose() {

    }

    KeyFramePose::KeyFramePose(KeyFrame *tKF) {
        this->KFPId = tKF->mnId;
        this->KFPose = tKF->GetPose();
        this->mConnectedKeyFrameWeights = tKF->mConnectedKeyFrameWeights;
        this->mvpMapPoints = tKF->mvpMapPoints;
    }

    MapPointPose::MapPointPose() {

    }

    MapPointPose::MapPointPose(MapPoint *tMP) {
        this->MPPId = tMP->mnId;
        this->mObservations = tMP->mObservations;
        this->MPPose = tMP->GetWorldPos();
    }

}