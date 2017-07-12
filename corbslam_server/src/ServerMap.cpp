//
// Created by lifu on 6/8/17.
//

#include "ServerMap.h"

namespace CORBSLAM_SERVER{

    ServerMap::ServerMap() {

    }

    ServerMap::ServerMap(Cache *tCache, Map *tMap) {

        this->pCacher = tCache;
        this->pMap = tMap;

    }

    std::vector<KeyFrame*> ServerMap::DetectMapFusionCandidatesFromDB(KeyFrame *tKF) {

        // get the map candidates from the serverMap in tKF using DBoW method

        std::vector<KeyFrame*> cands;

        cands = this->pCacher->mpKeyFrameDatabase->DetectMapFusionCandidatesFromDB( tKF );

        return cands;

    }

    void ServerMap::clear() {

        this->pMap->clear();
        this->pCacher->clearKeyframeDatabase();

    }

}