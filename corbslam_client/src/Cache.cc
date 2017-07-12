//
// Created by lifu on 17-1-31.
//

#include "System.h"
#include "Cache.h"
#include "Converter.h"
#include "DataDriver.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <time.h>
#include "ros/ros.h"

namespace ORB_SLAM2 {

    //construct function
    Cache::Cache() {

        mpMap = nullptr;
        mpKeyFrameDatabase = nullptr;
        mpVocabulary = nullptr;
        lKFToKFmap.clear();
        lMPToMPmap.clear();
        mbFinished = false;
        mlNewKeyFrames.clear();
        mbStopRequested = false;
        mbNotStop = true;
        mbStopped = false;
        mbFinishRequested = false;
        kfStatus.clear();

        pClientId = -1;

    }

    bool Cache::loadORBVocabulary(const string &strVocFile) {

        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if (!bVocLoad) {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }

        cout << "Vocabulary loaded!" << endl << endl;

        return bVocLoad;

    }

    void Cache::createKeyFrameDatabase() {
        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    }

    void Cache::createMap() {
        //Create the Map
        mpMap = new Map();
    }

    void Cache::AddKeyFrameToMap(KeyFrame *pKF) {

            {
                // add KeyFrame to the LinghtKeyFrame to KeyFrame map
                unique_lock<mutex> lock(mMutexlKFToKFmap);
                //unique_lock<mutex> lock2(mpMap->mMutexMapUpdate);
                mpMap->AddKeyFrame(pKF);
                lKFToKFmap[pKF->mnId] = pKF;
                kfStatus[pKF->mnId] = KF_IN_CACHE;
                tmpKFMap.erase(pKF->mnId);

            }
            {
                unique_lock<mutex> lock(mMutexNewKFs);
                mlNewKeyFrames.push_back(pKF);
            }
            {
                unique_lock<mutex> lock(updateMPsToServerMutex);
                newInsertedKFs.insert(LightKeyFrame(pKF));
            }

    }

    void Cache::AddKeyFrameFromServer(KeyFrame *pKF) {
        {
            // add KeyFrame to the LinghtKeyFrame to KeyFrame map
            unique_lock<mutex> lock(mMutexlKFToKFmap);
            //unique_lock<mutex> lock2(mpMap->mMutexMapUpdate);
            mpMap->AddKeyFrame(pKF);
            lKFToKFmap[pKF->mnId] = pKF;
            kfStatus[pKF->mnId] = KF_IN_CACHE;
            tmpKFMap.erase(pKF->mnId);

        }
        {
            unique_lock<mutex> lock(mMutexNewKFs);
            mlNewKeyFrames.push_back(pKF);
        }

    }

    void Cache::AddMapPointFromServer(MapPoint *pMP) {

        mpMap->AddMapPoint(pMP);

        // add MapPoint to MapPoint map
        unique_lock<mutex> lock(mMutexMPToMPmap);
        lMPToMPmap[pMP->mnId] = pMP;

    }

    void Cache::EraseKeyFrameFromMap(KeyFrame *pKF) {

        // erase KeyFrame to the LinghtKeyFrame to KeyFrame
        {
            //unique_lock<mutex> lock2(mpMap->mMutexMapUpdate);
            unique_lock<mutex> lock(mMutexlKFToKFmap);
            lKFToKFmap.erase(pKF->mnId);
            kfStatus.erase(pKF->mnId);
        }

        mpMap->EraseKeyFrame(pKF);

    }

    void Cache::AddMapPointToMap(MapPoint *pMP) {

            mpMap->AddMapPoint(pMP);

            // add MapPoint to MapPoint map
            unique_lock<mutex> lock(mMutexMPToMPmap);
            lMPToMPmap[pMP->mnId] = pMP;

            {
                unique_lock<mutex> lock(updateMPsToServerMutex);
                newInsertedMPs.insert(LightMapPoint(pMP));
            }

    }

    void Cache::EraseMapPointFromMap(MapPoint *pMP) {

        {
            unique_lock<mutex> lock(mMutexMPToMPmap);
            //cout << "erase mp : " << pMP->mnId << "\n";
            lMPToMPmap.erase(pMP->mnId);
        }

        mpMap->EraseMapPoint(pMP);

    }

    std::map<long unsigned int, MapPoint *> Cache::getlMPToMPmap() {
        return lMPToMPmap;
    }

    KeyFrame *Cache::getKeyFrameById(long unsigned int pId) {

        if (pId <= 0) return nullptr;

        if (kfStatus.find(pId) == kfStatus.end()) return nullptr;

        KeyFrame *pKF = nullptr;

        {
            if (tmpKFMap.find(pId) != tmpKFMap.end()) {
                pKF = tmpKFMap[pId];
            } else if (lKFToKFmap.find(pId) != lKFToKFmap.end())
                pKF = lKFToKFmap[pId];

        }
        return pKF;
    }

    bool Cache::KeyFrameInCache(long unsigned int pID) {

        if (kfStatus.find(pID) != kfStatus.end())
            return kfStatus[pID] == KF_IN_CACHE;

        return false;

    }

    MapPoint *Cache::getMapPointById(long unsigned int pId) {

        if (pId <= 0) return nullptr;

        MapPoint *pMP = nullptr;
        {
            //unique_lock<mutex> lock(mMutexMPToMPmap);
            if (lMPToMPmap.find(pId) != lMPToMPmap.end()) {
                unique_lock<mutex> lock(mMutexMPToMPmap);
                pMP = lMPToMPmap[pId];
            }
        }

        return pMP;

    }

    std::vector<MapPoint *> Cache::GetAllMapPointsFromMap() {
        return mpMap->GetAllMapPoints();
    }

    void Cache::SetReferenceMapPointsToMap(std::vector<MapPoint *> pLocalMapPoints) {
        mpMap->SetReferenceMapPoints(pLocalMapPoints);
    }

    void Cache::SetmvpKeyFrameOrigins(KeyFrame *pKF) {
        mpMap->mvpKeyFrameOrigins.push_back(pKF);
    }

    vector<KeyFrame *> Cache::getmvpKeyFrameOrigins() {
        return mpMap->mvpKeyFrameOrigins;
    }

    void Cache::addUpdateKeyframe(KeyFrame *tKF) {
        if (tKF) {
            if (tKF->getFixed())
                return;
            else {
                unique_lock<mutex> lock(updateMPsToServerMutex);
                LightKeyFrame tLKF(tKF);
                if (newInsertedKFs.find(tLKF) == newInsertedKFs.end())
                    updateKFs.insert(tLKF);
            }
        }
    }

    void Cache::addUpdateMapPoint(MapPoint *tMP) {

        if (tMP) {
            if (tMP->getFixed()) {
                return;
            } else {
                unique_lock<mutex> lock(updateMPsToServerMutex);
                LightMapPoint tLMP(tMP);
                if (newInsertedMPs.find(tLMP) == newInsertedMPs.end())
                    updateMPs.insert(tLMP);
            }
        }
    }

    const int Cache::getKeyFramesInMap() {
        return mpMap->KeyFramesInMap();
    }

    vector<KeyFrame *> Cache::getAllKeyFramesInMap() {
        return mpMap->GetAllKeyFrames();
    }

    long unsigned int Cache::GetMaxKFidInMap() {
        return mpMap->GetMaxKFid();
    }

    void Cache::clearMap() {

        mpMap->clear();
        {
            unique_lock<mutex> lock(mMutexlKFToKFmap);
            lKFToKFmap.clear();
        }
        lMPToMPmap.clear();
        mlNewKeyFrames.clear();
        tmpKFMap.clear();
        kfStatus.clear();
    }

    void Cache::EraseKeyFrameFromDB(KeyFrame *pKF) {
        mpKeyFrameDatabase->erase(pKF);
    }

    vector<KeyFrame *> Cache::DetectRelocalizationCandidatesFromDB(Frame *F) {

        return mpKeyFrameDatabase->DetectRelocalizationCandidates(F);

    }

    vector<KeyFrame *> Cache::DetectLoopCandidatesFromDB(KeyFrame *pKF, float minScore) {
        return mpKeyFrameDatabase->DetectLoopCandidates(pKF, minScore);
    }

    void Cache::clearKeyframeDatabase() {

        mpKeyFrameDatabase->clear();
    }


    void Cache::addKeyFrametoDB(KeyFrame *pKF) {

        mpKeyFrameDatabase->add(pKF);

    }

    void Cache::SaveMap(const string &filename) {
        // save Map to files
        std::ofstream ofs("savetest.txt");
        boost::archive::text_oarchive oa(ofs);
        oa << mpMap;
        ofs.close();

    }

    void Cache::LoadMap(const string &filename) {
        // loadMap to files

        std::ifstream ifs("savetest.txt");
        boost::archive::text_iarchive ia(ifs);
        ia >> mpMap;
        ifs.close();

    }

    /*  cache organize function   */

    void Cache::runUpdateToServer() {


        while (true) {

            std::set<LightKeyFrame> tnewInsertedKFs;
            std::set<LightMapPoint> tnewInsertedMPs;
            std::set<LightKeyFrame> tupdateKFs;
            std::set<LightMapPoint> tupdateMPs;
            {
                unique_lock<mutex> lock(updateMPsToServerMutex);

                tnewInsertedKFs = newInsertedKFs;
                tnewInsertedMPs = newInsertedMPs;
                tupdateKFs = updateKFs;
                tupdateMPs = updateMPs;

                newInsertedMPs.clear();
                newInsertedKFs.clear();
                updateKFs.clear();
                updateMPs.clear();

            }

            insertNewKeyFramesToServer(tnewInsertedKFs);


            insertNewMapPointsToServer(tnewInsertedMPs);


            updateKeyFramePosesToServer(tupdateKFs);


            updateMapPointPosesToServer(tupdateMPs);


            if (isStopped()) {
                // Safe area to stop
                while (isStopped() && !CheckFinish()) {
                    usleep(3000);
                }
                if (CheckFinish())
                    break;
            }

            if (CheckFinish())
                break;

            usleep(6000000);

        }

        SetFinish();
    }

    void Cache::insertNewKeyFramesToServer(std::set<LightKeyFrame> tInsertKFs) {

        ROS_INFO("CacheUpdate : update KFs to server");

        DataDriver DB(this);

        DB.insertNewKeyFramesToServer(tInsertKFs);

    }

    void Cache::insertNewMapPointsToServer(std::set<LightMapPoint> tInsertMPs) {

        ROS_INFO("CacheUpdate : update MPs to server");

        DataDriver DB(this);

        DB.insertNewMapPointsToServer(tInsertMPs);

    }

    void Cache::updateKeyFramePosesToServer(std::set<LightKeyFrame> tUpdateKFs) {

        ROS_INFO("CacheUpdate : update poses of KFs to server");

        DataDriver DB(this);

        DB.updateKeyFramePosesToServer(tUpdateKFs);

    }

    void Cache::updateMapPointPosesToServer(std::set<LightMapPoint> tUpdateMPs) {

        ROS_INFO("CacheUpdate : update poses of MPs to server");

        DataDriver DB(this);

        DB.updateMapPointPosesToServer(tUpdateMPs);


    }

    void Cache::runSubFromServer() {

        //TODO: sub using ROS SUB

        ros::NodeHandle n;

        //subcribe the add new KFs Topic published by Server
        ros::Subscriber subInsertKeyFrameToMap = n.subscribe("insertKeyFramesFromServer",
                                                             100, &ORB_SLAM2::Cache::subNewInsertKeyFramesFromServer,
                                                             this);

        //subcribe the add new MPs Topic published by Server
        ros::Subscriber subInsertMapPointToMap = n.subscribe("insertMapPointsFromServer",
                                                             100, &ORB_SLAM2::Cache::subNewInsertMapPointFromServer,
                                                             this);

        //subcribe the update poses of KFs Topic published by Server
        ros::Subscriber subUpdateKeyFrameToMap = n.subscribe("updateKeyFramePosesFromServer",
                                                             100, &ORB_SLAM2::Cache::subUpdatedKeyFramesPose, this);

        //subcribe the update poses of MPs Topic published by Server
        ros::Subscriber subUpdateMapPointToMap = n.subscribe("updateMapPointPosesFromServer",
                                                             100, &ORB_SLAM2::Cache::subUpdatedMapPointsPose, this);

        ros::spin();

    }

    void Cache::subNewInsertKeyFramesFromServer(const corbslam_server::corbslam_message::ConstPtr &msg) {

        ROS_INFO("sub New Insert KeyFrames From Server");

        std::map<int, cv::Mat> transMs;
        std::stringstream iiis(msg->TRANSM);
        boost::archive::text_iarchive iiia(iiis);
        iiia >> transMs;
        iiis.clear();

        if( transMs.find( this->pClientId ) == transMs.end() )
            return ;
        cv::Mat Ttrans = transMs[ this->pClientId ];

        std::stringstream is(msg->DATA);
        boost::archive::text_iarchive ia(is);
        std::vector<string> elementVec;
        ia >> elementVec;
        is.clear();
        for (int mit = 0; mit < (int) elementVec.size(); mit++) {
            try {
                ORB_SLAM2::KeyFrame *tKF = new ORB_SLAM2::KeyFrame();
                std::stringstream iis(elementVec[mit]);
                boost::archive::text_iarchive iia(iis);
                iia >> tKF;
                if (tKF && (tKF->mnClientId != this->pClientId)) {

                    if( this->getKeyFrameById(tKF->mnId) )
                        continue;

                    tKF->setCache(this);

                    cv::Mat Tcw = tKF->GetPose();
                    Tcw = Tcw * Ttrans.inv();
                    tKF->SetPose( Tcw );

                    tKF->setFixed();
                    this->AddKeyFrameFromServer(tKF);
                    this->addKeyFrametoDB(tKF);
                }
            } catch (...) {
                cout << "error in : " << mit << endl;
            }
        }
        elementVec.clear();

    }

    void Cache::subNewInsertMapPointFromServer(const corbslam_server::corbslam_message::ConstPtr &msg) {

        ROS_INFO("sub New Insert MapPoint From Server");

        std::map<int, cv::Mat> transMs;
        std::stringstream iiis(msg->TRANSM);
        boost::archive::text_iarchive iiia(iiis);
        iiia >> transMs;
        iiis.clear();

        if( transMs.find( this->pClientId ) == transMs.end() )
            return ;
        cv::Mat Ttrans = transMs[ this->pClientId ];

        std::stringstream is(msg->DATA);
        boost::archive::text_iarchive ia(is);
        std::vector<string> elementVec;
        ia >> elementVec;
        is.clear();

        cout << "get new map points from server : " << elementVec.size() << endl;
        for (int mit = 0; mit < (int) elementVec.size(); mit++) {
            try {
                ORB_SLAM2::MapPoint *tMP = new ORB_SLAM2::MapPoint();
                std::stringstream iis(elementVec[mit]);
                boost::archive::text_iarchive iia(iis);
                iia >> tMP;
                if (tMP && (tMP->mnClientId != this->pClientId)) {

                    if( this->getMapPointById( tMP->mnId ) )
                        continue;

                    tMP->setCache(this);

                    cv::Mat tPose = tMP->GetWorldPos();
                    cv::Mat tcw = Ttrans.rowRange(0, 3).col(3);
                    cv::Mat Rcw = Ttrans.rowRange(0, 3).colRange(0, 3);
                    tPose = Rcw * tPose + tcw ;
                    tMP->SetWorldPos( tPose );

                    tMP->setFixed();
                    this->AddMapPointFromServer(tMP);
                }
            } catch (...) {
                cout << "error in : " << mit << endl;
            }
        }
        elementVec.clear();

    }

    void Cache::subUpdatedKeyFramesPose(const corbslam_server::corbslam_message::ConstPtr &msg) {

        ROS_INFO("sub Updated KeyFrames Pose");

        std::map<int, cv::Mat> transMs;
        std::stringstream iiis(msg->TRANSM);
        boost::archive::text_iarchive iiia(iiis);
        iiia >> transMs;
        iiis.clear();

        if( transMs.find( this->pClientId ) == transMs.end() )
            return ;
        cv::Mat Ttrans = transMs[ this->pClientId ];

        std::vector<string> elementVec;
        elementVec.clear();
        std::stringstream is(msg->DATA);
        boost::archive::text_iarchive ia(is);
        ia >> elementVec;
        is.clear();

        for (int mit = 0; mit < (int) elementVec.size(); mit++) {
            try {
                ORB_SLAM2::KeyFramePose *tKFP = new ORB_SLAM2::KeyFramePose();
                std::stringstream iis(elementVec[mit]);
                boost::archive::text_iarchive iia(iis);
                iia >> tKFP;
                if (tKFP) {
                    ORB_SLAM2::KeyFrame *tKF = this->getKeyFrameById(tKFP->KFPId);
                    if (tKF && tKF->getFixed()) {
                        cv::Mat Tcw = tKFP->KFPose;
                        Tcw = Tcw * Ttrans.inv();
                        tKF->SetPose(Tcw);
                    }
                }
            } catch (...) {
                cout << "error in : " << mit << endl;
            }
        }

        elementVec.clear();

    }

    void Cache::subUpdatedMapPointsPose(const corbslam_server::corbslam_message::ConstPtr &msg) {

        ROS_INFO("sub Updated MapPoints Pose");

        std::map<int, cv::Mat> transMs;
        std::stringstream iiis(msg->TRANSM);
        boost::archive::text_iarchive iiia(iiis);
        iiia >> transMs;
        iiis.clear();

        if( transMs.find( this->pClientId ) == transMs.end() )
            return ;
        cv::Mat Ttrans = transMs[ this->pClientId ];

        std::vector<string> elementVec;
        elementVec.clear();
        std::stringstream is(msg->DATA);
        boost::archive::text_iarchive ia(is);
        ia >> elementVec;
        is.clear();

        for (int mit = 0; mit < (int) elementVec.size(); mit++) {
            try {
                ORB_SLAM2::MapPointPose *tMPP = new ORB_SLAM2::MapPointPose();
                std::stringstream iis(elementVec[mit]);
                boost::archive::text_iarchive iia(iis);
                iia >> tMPP;
                if (tMPP) {
                    ORB_SLAM2::MapPoint *tMP = this->getMapPointById(tMPP->MPPId);
                    if (tMP && tMP->getFixed() ) {

                        cv::Mat tPose = tMPP->MPPose;
                        cv::Mat tcw = Ttrans.rowRange(0, 3).col(3);
                        cv::Mat Rcw = Ttrans.rowRange(0, 3).colRange(0, 3);
                        tPose = Rcw * tPose + tcw;
                        tMP->SetWorldPos( tPose );
                    }
                }
            } catch (...) {
                cout << "error in : " << mit << endl;
            }
        }

        elementVec.clear();

    }


    void Cache::insertTmpKeyFrame(KeyFrame *pKF) {

        if (pKF) {

            unique_lock<mutex> lock(mMutexTmpKFMap);
            tmpKFMap[pKF->mnId] = pKF;
            kfStatus[pKF->mnId] = KF_IN_CACHE;

        }

    }

    void Cache::eraseTmpKeyFrame(long unsigned int pID) {
        unique_lock<mutex> lock(mMutexTmpKFMap);
        if (tmpKFMap.find(pID) != tmpKFMap.end()) {

            tmpKFMap.erase(pID);

        }
    }


    bool Cache::isStopped() {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Cache::CheckFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Cache::SetFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
        unique_lock<mutex> lock2(mMutexStop);
        mbStopped = true;
    }

    void Cache::RequestFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Cache::isFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    bool Cache::Stop() {
        unique_lock<mutex> lock(mMutexStop);
        if (mbStopRequested && !mbNotStop) {
            mbStopped = true;
            cout << "Cache STOP" << endl;
            return true;
        }

        return false;
    }

}