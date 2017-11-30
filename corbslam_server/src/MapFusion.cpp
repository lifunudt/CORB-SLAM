//
// Created by lifu on 6/6/17.
//

#include "corbslam_client/corbslam_insertRequest.h"
#include "corbslam_client/corbslam_message.h"
#include "corbslam_client/corbslam_insert.h"
#include "corbslam_client/corbslam_update.h"
#include "corbslam_client/corbslam_updateRequest.h"
#include <thread>
#include "MapFusion.h"

namespace CORBSLAM_SERVER{

    static std::vector< std::string > elementVec;

    MapFusion::MapFusion( std::string strSettingPath ){

        this->mpStrSettingPath = strSettingPath;

        memset( ifSubToGlobalMap, false, sizeof( ifSubToGlobalMap ));

        for( int i = 0 ; i < 100; i++ ) {
            subMapTransM[i] = cv::Mat::eye( 4,4, CV_32F );
        }

        ifNullGlobalMap = true;

    }

    bool MapFusion::insertKeyFrameToMap(corbslam_client::corbslam_insert::Request &req,
                                        corbslam_client::corbslam_insert::Response &res){

        ROS_INFO("--Server Client[%d]: insert keyframe to map func. KFs count: %d", req.CID, req.COUNT);

        int clientId = req.CID;

        unique_lock<mutex> lock( mStreamInOutPutMutex );

        elementVec.clear();
        std::stringstream is( req.DATA );
        boost::archive::text_iarchive ia(is);
        ia >> elementVec;
        {
            std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
            if( ifSubToGlobalMap[ clientId ] ) {

                for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                    try {
                        ORB_SLAM2::KeyFrame *tKF = new ORB_SLAM2::KeyFrame();
                        std::stringstream iis( elementVec[mit] );
                        boost::archive::text_iarchive iia(iis);
                        iia >> tKF;
                        if( tKF && !globalMap->pCacher->getKeyFrameById( tKF->mnId)) {
                            tKF->setCache( globalMap->pCacher );
                            cv::Mat Tcw = tKF->GetPose();
                            Tcw = Tcw * subMapTransM[ clientId ];
                            tKF->SetPose( Tcw );
                            tKF->ifFixed = false;
                            globalMap->pCacher->AddKeyFrameToMap( tKF );
                            globalMap->pCacher->addKeyFrametoDB( tKF );
                        }
                    } catch( ... ) {
                        cout << "error in : " << mit << endl;
                    }
                }

            } else {

                if( serverMap.find( clientId ) == serverMap.end() ) {
                    createSubServerMap( clientId );
                }

                ServerMap * clientMap = serverMap[ clientId ];
                clientMap->pCacher->pClientId = clientId;

                for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                    try {
                        ORB_SLAM2::KeyFrame *tKF = new ORB_SLAM2::KeyFrame();
                        std::stringstream iis( elementVec[mit] );
                        boost::archive::text_iarchive iia(iis);
                        iia >> tKF;
                        if( tKF && !clientMap->pCacher->getKeyFrameById( tKF->mnId)) {
                            tKF->setCache( clientMap->pCacher );
                            tKF->ifFixed = false;
                            clientMap->pCacher->AddKeyFrameToMap( tKF );
                            clientMap->pCacher->addKeyFrametoDB( tKF );
                        }
                    } catch( ... ) {
                        cout << elementVec[mit] << endl;
                        cout << "error in : " << mit << endl;
                    }
                }

            }

        }


        elementVec.clear();

        return true;

    }

    bool MapFusion::insertMapPointToMap(corbslam_client::corbslam_insert::Request &req,
                                        corbslam_client::corbslam_insert::Response &res){

        ROS_INFO("--Server Client[%d]: insert MapPoint to map func. MPs count: %d " ,req.CID, req.COUNT );

        int clientId = req.CID;
        unique_lock<mutex> lock( mStreamInOutPutMutex );
        elementVec.clear();
        std::stringstream is( req.DATA );
        boost::archive::text_iarchive ia(is);
        ia >> elementVec;
        {
            std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
            if( ifSubToGlobalMap[ clientId ] ) {
                for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                    try {
                        ORB_SLAM2::MapPoint *tMP = new ORB_SLAM2::MapPoint();
                        std::stringstream iis( elementVec[mit] );
                        boost::archive::text_iarchive iia(iis);
                        iia >> tMP;
                        if( tMP && !globalMap->pCacher->getMapPointById( tMP->mnId ) ) {
                            tMP->setCache( globalMap->pCacher );
                            cv::Mat tPose = tMP->GetWorldPos();
                            cv::Mat Tn2o = cv::Mat::eye(4,4, tPose.type());
                            cv::Mat tcw = subMapTransM[clientId].rowRange(0, 3).col(3);
                            cv::Mat Rwc = subMapTransM[clientId].rowRange(0, 3).colRange(0, 3).t();
                            tPose = Rwc * ( tPose - tcw );
                            tMP->SetWorldPos( tPose );
                            tMP->ifFixed = false;
                            globalMap->pCacher->AddMapPointToMap( tMP );
                        }
                    } catch ( ... ) {
                        cout << endl << "Exception in insert MapPoint: " << mit << endl;
                    }
                }
            } else {

                if( serverMap.find( clientId ) == serverMap.end() ) {
                    createSubServerMap( clientId );
                }
                ServerMap * clientMap = serverMap[ clientId ];
                clientMap->pCacher->pClientId = clientId;

                for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                    try {
                        ORB_SLAM2::MapPoint *tMP = new ORB_SLAM2::MapPoint();
                        std::stringstream iis( elementVec[mit] );
                        boost::archive::text_iarchive iia(iis);
                        iia >> tMP;
                        if( tMP && !clientMap->pCacher->getMapPointById( tMP->mnId )) {
                            tMP->setCache( clientMap->pCacher );
                            tMP->ifFixed = false;
                            clientMap->pCacher->AddMapPointToMap( tMP );
                        }
                    } catch ( ... ) {
                        cout << endl << "Exception in insert MapPoint: " << mit << endl;
                    }
                }
            }
        }

        elementVec.clear();

        return true;
    }

    bool MapFusion::updateKeyFrameToMap(corbslam_client::corbslam_update::Request &req,
                                        corbslam_client::corbslam_update::Response &res) {

        ROS_INFO("--Server client[%d]: update keyframe to map func. KFs count:%d" , req.CID, req.COUNT );

        int clientId = req.CID;

        unique_lock<mutex> lock( mStreamInOutPutMutex );

        if ( req.COUNT > 0 ) {

            elementVec.clear();
            std::stringstream is( req.DATA );
            boost::archive::text_iarchive ia(is);
            ia >> elementVec;

            {
                std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
                if( ifSubToGlobalMap[ clientId ] ) {
                    for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                        try {
                            ORB_SLAM2::KeyFramePose *tKFP = new ORB_SLAM2::KeyFramePose();
                            std::stringstream iis( elementVec[mit] );
                            boost::archive::text_iarchive iia(iis);
                            iia >> tKFP;
                            if( tKFP ) {
                                ORB_SLAM2::KeyFrame * tKF = globalMap->pCacher->getKeyFrameById( tKFP->KFPId );
                                if( tKF ) {
                                    cv::Mat Tcw = tKFP->KFPose;
                                    Tcw = Tcw * subMapTransM[ clientId ];
                                    tKF->SetPose( Tcw );
                                    globalMap->pCacher->addUpdateKeyframe( tKF );
                                }
                            }
                        } catch( ... ) {
                            cout << "error in : " << mit << endl;
                        }
                    }
                } else {
                    if( serverMap.find( clientId ) == serverMap.end() ) {
                        return false;
                    }

                    ServerMap * clientMap = serverMap[ clientId ];

                    for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                        try {
                            ORB_SLAM2::KeyFramePose *tKFP = new ORB_SLAM2::KeyFramePose();
                            std::stringstream iis( elementVec[mit] );
                            boost::archive::text_iarchive iia(iis);
                            iia >> tKFP;
                            if( tKFP ) {
                                ORB_SLAM2::KeyFrame * tKF = clientMap->pCacher->getKeyFrameById( tKFP->KFPId );
                                if( tKF ){
                                    tKF->SetPose( tKFP->KFPose );
                                }
                            }
                        } catch( ... ) {
                            cout << "error in : " << mit << endl;
                        }
                    }
                }
            }


            elementVec.clear();
        }

        return true;
    }

    bool MapFusion::updateMapPointToMap(corbslam_client::corbslam_update::Request &req,
                                        corbslam_client::corbslam_update::Response &res) {

        ROS_INFO("--Server Client[%d]: update MapPoint to map func. MPs count:%d", req.CID, req.COUNT );

        int clientId = req.CID;

        unique_lock<mutex> lock( mStreamInOutPutMutex );

        if ( req.COUNT > 0 ) {

            elementVec.clear();
            std::stringstream is( req.DATA );
            boost::archive::text_iarchive ia(is);
            ia >> elementVec;
            {
                std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
                if( ifSubToGlobalMap[ clientId ] ) {
                    for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                        try {
                            ORB_SLAM2::MapPointPose *tMPP = new ORB_SLAM2::MapPointPose();
                            std::stringstream iis( elementVec[mit] );
                            boost::archive::text_iarchive iia(iis);
                            iia >> tMPP;
                            if( tMPP ) {
                                ORB_SLAM2::MapPoint * tMP = globalMap->pCacher->getMapPointById( tMPP->MPPId );
                                if( tMP ) {
                                    cv::Mat tPose = tMPP->MPPose;
                                    cv::Mat tcw = subMapTransM[clientId].rowRange(0, 3).col(3);
                                    cv::Mat Rwc = subMapTransM[clientId].rowRange(0, 3).colRange(0, 3).t();
                                    tPose = Rwc * ( tPose - tcw );
                                    tMP->SetWorldPos( tPose );
                                    globalMap->pCacher->addUpdateMapPoint( tMP );
                                }
                            }
                        } catch( ... ) {
                            cout << "error in : " << mit << endl;
                        }
                    }
                } else {
                    if( serverMap.find( clientId ) == serverMap.end() ) {
                        return false;
                    }
                    ServerMap * clientMap = serverMap[ clientId ];

                    for ( int mit = 0 ; mit < (int)elementVec.size(); mit++ ) {
                        try {
                            ORB_SLAM2::MapPointPose *tMPP = new ORB_SLAM2::MapPointPose();
                            std::stringstream iis( elementVec[mit] );
                            boost::archive::text_iarchive iia(iis);
                            iia >> tMPP;
                            if( tMPP ) {
                                ORB_SLAM2::MapPoint * tMP = clientMap->pCacher->getMapPointById( tMPP->MPPId );
                                if( tMP ){
                                    tMP->SetWorldPos( tMPP->MPPose );
                                }
                            }
                        } catch( ... ) {
                            cout << "error in : " << mit << endl;
                        }
                    }
                }
            }


            elementVec.clear();
        }

        return true;

    }

    void MapFusion::runPubTopic() {

        ROS_INFO("Thread runPubInsertTopic start...");

        ros::Rate loop_rate(0.5);

        while( ros::ok() ) {
            //if not resent the global map tp the clients, publish the new or updated to clients
            {

                std::unique_lock<mutex> lock( resentGlobalMapMutex );

                std::set<LightKeyFrame> tnewInsertedKFs ;
                std::set<LightMapPoint> tnewInsertedMPs ;
                std::set<LightKeyFrame> tupdateKFs ;
                std::set<LightMapPoint> tupdateMPs ;

                {
                    std::unique_lock<mutex> lock(mSubMapUpdatedMutex);

                    tnewInsertedKFs = globalMap->pCacher->newInsertedKFs;
                    tnewInsertedMPs = globalMap->pCacher->newInsertedMPs;
                    tupdateKFs = globalMap->pCacher->updateKFs;
                    tupdateMPs = globalMap->pCacher->updateMPs;

                    globalMap->pCacher->newInsertedMPs.clear();
                    globalMap->pCacher->newInsertedKFs.clear();
                    globalMap->pCacher->updateKFs.clear();
                    globalMap->pCacher->updateMPs.clear();

                }
                int count = (int)( tnewInsertedKFs.size() + tnewInsertedMPs.size() + tupdateKFs.size() + tupdateMPs.size() );

                if( count > 0 ) {

                    ROS_INFO( "Publish to client count = %d", count);

                    pubToClient->pubNewKFsToClients( tnewInsertedKFs );
                    ros::spinOnce();
                    pubToClient->pubNewMPsToClients( tnewInsertedMPs );
                    ros::spinOnce();
                    pubToClient->pubUpdatedKFsToClients( tupdateKFs );
                    ros::spinOnce();
                    pubToClient->pubUpdatedMPSToClients( tupdateMPs );
                    ros::spinOnce();

                }

            }

            loop_rate.sleep();
        }

    }

    void MapFusion::resentGlobalMapToClient() {

        std::unique_lock<mutex> lock( resentGlobalMapMutex );

        ROS_INFO( "Resent to client start");

        std::vector<KeyFrame * > allKFs ;
        std::vector<MapPoint * > allMPs ;

        {
            std::unique_lock<mutex> lock1(mSubMapUpdatedMutex);

            globalMap->pCacher->newInsertedMPs.clear();
            globalMap->pCacher->newInsertedKFs.clear();
            globalMap->pCacher->updateKFs.clear();
            globalMap->pCacher->updateMPs.clear();

            allKFs = globalMap->pMap->GetAllKeyFrames();
            allMPs = globalMap->pMap->GetAllMapPoints();

        }

        int mit = 0;

        ros::Rate loop_rate(0.5);

        time_t start_t = clock();

        while( ros::ok() ) {

            std::set<LightKeyFrame> tmpLKFs;
            std::set<LightMapPoint> tmpLMPs;

            for( int i = mit * 50 ; i < min( (int)allKFs.size(), (mit + 1 ) * 50 ); i++ ) {
                tmpLKFs.insert( LightKeyFrame( allKFs[i]));
            }

            for( int i = mit * 2000 ; i < min( (int)allMPs.size(), (mit + 1 ) * 2000 ); i++ ) {
                tmpLMPs.insert( LightMapPoint( allMPs[i]));
            }

            if( tmpLKFs.size() == 0 && tmpLMPs.size() == 0) {
                break;
            } else {

                pubToClient->pubNewKFsToClients( tmpLKFs );
                ros::spinOnce();
                pubToClient->pubNewMPsToClients( tmpLMPs );
                ros::spinOnce();
                mit++;
            }

            loop_rate.sleep();

        }

        time_t end_t = clock();

        ROS_INFO( "Resent to client end, use time: %lfs", (double)(end_t - start_t)/(double)CLOCKS_PER_SEC );

    }

    void MapFusion::fuseSubMapToMap(){

        ros::Rate loop_rate(0.5);

        while( ros::ok() ) {

            for (std::map<int, ServerMap *>::iterator tmpx = serverMap.begin(); tmpx != serverMap.end(); tmpx++) {

                //TODO the map fusion logic need to be fixed
                if( !ifSubToGlobalMap[ (*tmpx).first ] ) {

                    mapFuseToGlobalMap( (*tmpx).second );

                }
            }

            ros::spinOnce();

            loop_rate.sleep();

        }

    }

    bool MapFusion::mapFuseToGlobalMap(ServerMap *sMap) {

        time_t start_t = clock();
        fstream detectf;
        detectf.open( "detectMap.txt" );

        // if global map is null, this submap is inserted into the global map and does not do transform
        {
            std::unique_lock<mutex> lock( nullGlobalMapMutex );
            if( ifNullGlobalMap ) {

                cv::Mat Tnorm = cv::Mat::eye(4,4, CV_32F);
                std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
                insertServerMapToGlobleMap(sMap, Tnorm);
                ifSubToGlobalMap[(*sMap).pCacher->pClientId] = true;
                subMapTransM[(*sMap).pCacher->pClientId] = Tnorm;
                pubToClient->transMs[ (*sMap).pCacher->pClientId ] = Tnorm;
                sMap->clear();
                ifNullGlobalMap = false;

                cout <<"Global Map is not null!\n";
                return true;
            }
        }

        bool flag = false;
        std::vector<KeyFrame*> allKeyFramesInMapy = sMap->pMap->GetAllKeyFrames();

        bool bOK = false;
        std::vector<KeyFrame *> candidateKFs;
        KeyFrame * currentKF;

        for( int mit = 0; mit < (int)allKeyFramesInMapy.size(); mit++ ) {

            KeyFrame * tKF = allKeyFramesInMapy[mit];

            cv::Mat oldTwc = tKF->GetPoseInverse();
            cv::Mat oldTcw = tKF->GetPose();
            cv::Mat newTcw = cv::Mat::eye(4,4, newTcw.type());

            candidateKFs.clear();
            currentKF = tKF;

            bOK = detectKeyFrameInServerMap( globalMap, tKF, newTcw, candidateKFs);

            if( bOK ) {

                mpGBA->setCurentKeyFrame( currentKF );

                mpGBA->setCandidates( candidateKFs );

                if( mpGBA->ComputeSim3() ) {

                    ROS_INFO("Detect In serverMap[%d], from keyframe id[%d]", (int)sMap->pCacher->pClientId, (int)tKF->mnId);
                    // if this keyframe is deteted in the serverMapx

                    cv::Mat To2n = oldTwc * newTcw;
                    cv::Mat Tnorm = cv::Mat::eye(4,4, newTcw.type());

                    flag = true;

                    {
                        std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
                        subMapTransM[ (*sMap).pCacher->pClientId ] = To2n;
                        pubToClient->transMs[ (*sMap).pCacher->pClientId ] = To2n;
                        ifSubToGlobalMap[ (*sMap).pCacher->pClientId ] = true;
                        insertServerMapToGlobleMap( sMap, To2n );
                        sMap->clear();
                    }

                    mpGBA->CorrectLoop();

                    time_t end_t = clock();

                    cout << "mapfuse " << globalMap->pMap->KeyFramesInMap() << " " << allKeyFramesInMapy.size() << " " << (double)( end_t - start_t )/(double)CLOCKS_PER_SEC << endl;

                    break;

                }

            }

        }

        if( flag ){
            resentGlobalMapToClient();
        }
        return flag;

    }

    bool MapFusion::mapFuse( ServerMap* sMapx, ServerMap* sMapy) {

        bool flag = false;
        std::vector<KeyFrame*> allKeyFramesInMapy = sMapy->pMap->GetAllKeyFrames();

        bool bOK = false;
        std::vector<KeyFrame *>  candidateKFs;
        KeyFrame * currentKF;

        for( int mit = 0; mit < (int)allKeyFramesInMapy.size(); mit++ ) {

            KeyFrame * tKF = allKeyFramesInMapy[mit];

            cv::Mat oldTwc = tKF->GetPoseInverse();
            cv::Mat oldTcw = tKF->GetPose();
            cv::Mat newTcw = cv::Mat::eye(4,4, newTcw.type());

            time_t start_t = clock();

            candidateKFs.clear();
            currentKF = tKF;

            bOK = detectKeyFrameInServerMap( sMapx, tKF, newTcw, candidateKFs);

            if( bOK ) {

                mpGBA->setCurentKeyFrame( currentKF );

                mpGBA->setCandidates( candidateKFs );

                if( mpGBA->ComputeSim3() ) {

                    ROS_INFO("Detect In serverMap[%d], from keyframe id[%d]", (int)sMapx->pCacher->pClientId, (int)tKF->mnId);
                    // if this keyframe is deteted in the serverMapx

                    start_t = clock();

                    cv::Mat To2n = oldTwc * newTcw;
                    cv::Mat Tnorm = cv::Mat::eye(4,4, newTcw.type());

                    flag = true;
                    {
                        std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
                        insertServerMapToGlobleMap(sMapx, Tnorm);
                        ifSubToGlobalMap[(*sMapx).pCacher->pClientId] = true;
                        subMapTransM[(*sMapx).pCacher->pClientId] = Tnorm;
                        pubToClient->transMs[ (*sMapx).pCacher->pClientId ] = Tnorm;
                        sMapx->clear();
                    }
                    {
                        std::unique_lock<mutex> lock(mSubMapUpdatedMutex);
                        insertServerMapToGlobleMap( sMapy, To2n );
                        ifSubToGlobalMap[ (*sMapy).pCacher->pClientId ] = true;
                        subMapTransM[ (*sMapy).pCacher->pClientId ] = To2n;
                        pubToClient->transMs[ (*sMapy).pCacher->pClientId ] = To2n;
                        sMapy->clear();
                    }

                    time_t end_t = clock();
                    cout << "Fusing map use time is : " << ( double )( end_t - start_t ) / (double)CLOCKS_PER_SEC << "s \n";

                    mpGBA->CorrectLoop();

                    break;

                }

            }

        }

        return flag;

    }

    void MapFusion::insertServerMapToGlobleMap(ServerMap *sMap, cv::Mat To2n) {

        vector<KeyFrame * > allKFs = sMap->pMap->GetAllKeyFrames();

        vector<MapPoint * > allMPs = sMap->pMap->GetAllMapPoints();

        for( int mit = 0; mit < (int)allKFs.size(); mit ++ ) {

            KeyFrame * tKF = allKFs[ mit ];
            cv::Mat Tcw = tKF->GetPose();

            Tcw = Tcw * To2n;

            tKF->SetPose( Tcw );
            tKF->setCache( globalMap->pCacher );
            globalMap->pCacher->AddKeyFrameToMap( tKF );
            globalMap->pCacher->addKeyFrametoDB( tKF );

        }

        for( int mit = 0 ; mit < (int)allMPs.size(); mit ++ ) {

            MapPoint * tMP = allMPs[ mit ];

            cv::Mat tPose = tMP->GetWorldPos();
            cv::Mat Tn2o = cv::Mat::eye(4,4, To2n.type());
            cv::Mat tcw = To2n.rowRange(0, 3).col(3);
            cv::Mat Rwc = To2n.rowRange(0, 3).colRange(0, 3).t();
            tPose = Rwc * ( tPose - tcw );

            tMP->SetWorldPos( tPose );
            tMP->setCache( globalMap->pCacher );
            globalMap->pCacher->AddMapPointToMap( tMP );

        }

    }

    bool MapFusion::detectKeyFrameInServerMap(ServerMap *sMap, KeyFrame *tKF, cv::Mat &newPose, std::vector<KeyFrame *> &candidateKFs) {

        std::vector<KeyFrame*> vpCandidateKFs = sMap->DetectMapFusionCandidatesFromDB( tKF );

        if(vpCandidateKFs.empty())
            return false;

        const int nKFs = (int) vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75,true);

        vector<PnPsolver*> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        vector<vector<MapPoint*> > vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates=0;

        for(int i=0; i<nKFs; i++)
        {
            KeyFrame* pKF = vpCandidateKFs[i];
            if(pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                int nmatches = matcher.SearchByBoWInServer(pKF,tKF,vvpMapPointMatches[i]);
                if(nmatches<15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    PnPsolver* pSolver = new PnPsolver( (*tKF) ,vvpMapPointMatches[i]);

                    pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                    candidateKFs.push_back( pKF );
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9,true);

        while(nCandidates>0 && !bMatch)
        {
            for(int i=0; i<nKFs; i++)
            {
                if(vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                PnPsolver* pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if(bNoMore)
                {
                    vbDiscarded[i]=true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if(!Tcw.empty())
                {
                    Tcw.copyTo(newPose);

                    bMatch = true;
                    break;
                }
            }
        }

        if(!bMatch)
        {
            return false;
        }
        else
        {
            return true;
        }

    }


    void MapFusion::createSubServerMap(int mapId) {

        Cache * pCache = new Cache();

        pCache->mpVocabulary = mpVocabulary;

        pCache->createKeyFrameDatabase();

        pCache->createMap();

        ServerMap * subMap = new ServerMap( pCache, pCache->getMpMap() );

        serverMap[ mapId ] = subMap;

    }

    bool MapFusion::loadORBVocabulary(const string &strVocFile) {

        //Load ORB Vocabulary

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if (!bVocLoad) {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }

        ROS_INFO( "Vocabulary loaded!" );

        return bVocLoad;

    }

    void MapFusion::createKeyFrameDatabase() {
        //create new global server Map

        this->mpKeyFrameDatabase = new KeyFrameDatabase( *mpVocabulary );

        Cache * pCache = new Cache();

        pCache->mpVocabulary = mpVocabulary;

        pCache->createKeyFrameDatabase();

        pCache->createMap();

        pCache->pClientId = 0;

        globalMap = new ServerMap( pCache, pCache->getMpMap() );

        pubToClient = new PubToClient( globalMap->pCacher );

        //global map view

        mpGlobalMapDrawer = new MapDrawer( pCache->getMpMap(), mpStrSettingPath );

        mpSMView = new ServerMapView(mpGlobalMapDrawer, mpStrSettingPath);

        mptViewer = new thread(&ServerMapView::Run, mpSMView);

        mpGBA = new GlobalOptimize( globalMap );

    }


}
