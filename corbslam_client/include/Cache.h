//
// Created by lifu on 17-1-31.
//

#ifndef ORB_SLAM2_CACHE_H
#define ORB_SLAM2_CACHE_H

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "SerializeObject.h"
#include "LightKeyFrame.h"
#include "LightMapPoint.h"

#include "ros/ros.h"

#include "corbslam_client/corbslam_insert.h"
#include "corbslam_client/corbslam_update.h"
#include "corbslam_client/corbslam_message.h"
#include "TransPose.h"


/*
 * Cache class is the data layer which organize all the persistent data, such as keyframe, mappoint, all kinds of map.
 * Cache class provide the all metheds which access the data layer such as basic read and store methods
 *
 */

namespace ORB_SLAM2 {

    class Map;

    // class ORBVocabulary;
    class KeyFrameDatabase;

    class KeyFrame;

    class LightKeyFrame;

    class LightMapPoint;

    class KeyFramePose;

    class MapPointPose;

    enum KF_status { KF_IN_CACHE, KF_IN_SERVER, KF_UNEXIST };

    class Cache {

    public:
        //construct function
        Cache();

        //load the ORBVocabulary from the file
        bool loadORBVocabulary(const string &strVocFile);

        void createKeyFrameDatabase();

        void createMap();

        void runUpdateToServer();

        void runSubFromServer();

        // operate about keyframes

        // add one keyFrame to map
        void AddKeyFrameToMap(KeyFrame *pKF);

        //erase one particular keyFrame in map
        void EraseKeyFrameFromMap(KeyFrame *pKF);

        //get the number of keyframes in map
        const int getKeyFramesInMap();

        //get all keyframes in the map
        vector<KeyFrame *> getAllKeyFramesInMap();

        long unsigned int GetMaxKFidInMap();

        KeyFrame *getKeyFrameById(long unsigned int pId);

        MapPoint *getMapPointById(long unsigned int pId);

        // operate about mappoint
        //add one MapPoint to map
        void AddMapPointToMap(MapPoint *pMP);

        // erase the paticular mapPoint from the map
        void EraseMapPointFromMap(MapPoint *pMP);

        // get all MapPoints in the map
        std::vector<MapPoint *> GetAllMapPointsFromMap();

        //set ReferenceMapPoint in the map
        void SetReferenceMapPointsToMap(std::vector<MapPoint *> pLocalMapPoints);

        //push back keyFrame to mvpKeyFrameOrigin in Map
        void SetmvpKeyFrameOrigins(KeyFrame *pKF);

        //get mvpKeyFrameOrigins from map
        vector<KeyFrame *> getmvpKeyFrameOrigins();

        void addUpdateKeyframe( KeyFrame * tKF);
        void addUpdateMapPoint( MapPoint * tMP);

        //Clear map
        void clearMap();

        //operate about keyframeDatabase
        //erase one keyfrmae from keyFramedatabase
        void EraseKeyFrameFromDB(KeyFrame *pKF);

        //KeyFrameDatabase detectRelocalizationCandidates keyframes
        vector<KeyFrame *> DetectRelocalizationCandidatesFromDB(Frame *F);

        //KeyFrameDatabase detect the loopCandi keyframes
        vector<KeyFrame *> DetectLoopCandidatesFromDB(KeyFrame *pKF, float minScore);

        //add one keyFrame to keyFrameDatabase
        void addKeyFrametoDB(KeyFrame *pKF);

        //clear KeyFrameDatabase
        void clearKeyframeDatabase();

        //get and set functions
        //get ORBVocabulary point
        ORBVocabulary *getMpVocabulary() const {
            return mpVocabulary;
        }

        //get the keyFrameDatabase point
        KeyFrameDatabase *getMpKeyFrameDatabase() const {
            return mpKeyFrameDatabase;
        }

        //get the map point
        Map *getMpMap() const {
            return mpMap;
        }

        std::map<long unsigned int, MapPoint *> getlMPToMPmap();

        void SaveMap(const string &filename);

        void LoadMap(const string &filename);

        void RequestFinish();

        bool isFinished();

        void insertTmpKeyFrame( KeyFrame* pKF);

        void eraseTmpKeyFrame( long unsigned int pID );

        bool KeyFrameInCache( long unsigned int pID);

        std::mutex mMutexCacheMap;

        int pClientId;

        // ORB vocabulary used for place recognition and feature matching.
        ORBVocabulary *mpVocabulary;

        // KeyFrame database for place recognition (relocalization and loop detection).
        KeyFrameDatabase *mpKeyFrameDatabase;

        // Map structure that stores the pointers to all KeyFrames and MapPoints.
        Map *mpMap;

        std::set<LightKeyFrame> newInsertedKFs;
        std::set<LightMapPoint> newInsertedMPs;
        std::set<LightKeyFrame> updateKFs;
        std::set<LightMapPoint> updateMPs;

    private:
        /*  cache organize function   */

        bool Stop();
        bool isStopped();
        bool CheckFinish();
        void SetFinish();

        void AddKeyFrameFromServer(KeyFrame *pKF);
        void AddMapPointFromServer(MapPoint *pMP);

        void insertNewKeyFramesToServer( std::set<LightKeyFrame> tInsertKFs);
        void insertNewMapPointsToServer( std::set<LightMapPoint> tInsertMPs);

        void updateKeyFramePosesToServer( std::set<LightKeyFrame> tUpdateKFs );
        void updateMapPointPosesToServer( std::set<LightMapPoint> tUpdateMPs );

        void subNewInsertKeyFramesFromServer(  const corbslam_client::corbslam_message::ConstPtr& msg  );
        void subNewInsertMapPointFromServer(  const corbslam_client::corbslam_message::ConstPtr& msg  );

        void subUpdatedKeyFramesPose(  const corbslam_client::corbslam_message::ConstPtr& msg  );
        void subUpdatedMapPointsPose(  const corbslam_client::corbslam_message::ConstPtr& msg  );


    private:

        std::mutex mMutexlKFToKFmap;
        std::map<long unsigned int, KeyFrame *> lKFToKFmap;

        std::mutex mMutexMPToMPmap;
        std::map<long unsigned int, MapPoint *> lMPToMPmap;

        //cache orgnize data

        std::mutex updateMPsToServerMutex;



        bool mbFinished;

        std::mutex mMutexNewKFs;
        vector<KeyFrame* > mlNewKeyFrames;

        std::mutex mMutexStop;
        bool mbStopRequested;
        bool mbNotStop;
        bool mbStopped;

        std::mutex mMutexFinish;
        bool mbFinishRequested;

        std::mutex mMutexTmpKFMap;

        std::map<long unsigned int, KeyFrame*> tmpKFMap;

        std::map<long unsigned int, KF_status > kfStatus;


    };

} //namespace ORB_SLAM
#endif //ORB_SLAM2_CACHE_H

