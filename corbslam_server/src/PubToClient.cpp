//
// Created by lifu on 6/19/17.
//

#include "corbslam_client/corbslam_message.h"
#include "PubToClient.h"

namespace CORBSLAM_SERVER{

    PubToClient::PubToClient(Cache * pCache) {

        this->pCacher = pCache;
        this->transMs.clear();

        ros::NodeHandle n;

        insertKeyFramesPub = n.advertise<corbslam_client::corbslam_message>("insertKeyFramesFromServer", 1000);
        insertMapPointsPub = n.advertise<corbslam_client::corbslam_message>("insertMapPointsFromServer", 1000);
        updateKeyFramePosesPub = n.advertise<corbslam_client::corbslam_message>("updateKeyFramePosesFromServer", 1000);
        updateMapPointPosesPub = n.advertise<corbslam_client::corbslam_message>("updateMapPointPosesFromServer", 1000);

    }

    void PubToClient::pubNewKFsToClients( std::set<LightKeyFrame> newKFs ) {

        corbslam_client::corbslam_message msg;
        std::vector< std::string > KFsData;
        KFsData.clear();

        for( std::set<LightKeyFrame>::iterator mit = newKFs.begin(); mit != newKFs.end(); mit++ ) {

            KeyFrame * tKF = (*mit).getKeyFrame();
            if( tKF ) {
                std::ostringstream os;
                boost::archive::text_oarchive oa(os);
                oa << tKF;
                KFsData.push_back( os.str() );
                os.clear();
            }
        }
        std::ostringstream os;
        boost::archive::text_oarchive oa(os);
        oa << KFsData;
        msg.DATA = os.str();
        os.clear();
        KFsData.clear();

        std::ostringstream oos;
        boost::archive::text_oarchive ooa(oos);
        ooa << transMs;
        msg.TRANSM = oos.str();
        oos.clear();

        insertKeyFramesPub.publish(msg);

    }

    void PubToClient::pubNewMPsToClients( std::set<LightMapPoint> newMPs ) {

        corbslam_client::corbslam_message msg;

        std::vector< std::string > MPsData;
        MPsData.clear();
        for( std::set<LightMapPoint>::iterator mit = newMPs.begin(); mit != newMPs.end(); mit++ ) {

            MapPoint * tMP = (*mit).getMapPoint();
            if( tMP ) {
                std::ostringstream os;
                boost::archive::text_oarchive oa(os);
                oa << tMP;
                MPsData.push_back( os.str() );
                os.clear();
            }
        }
        std::ostringstream os;
        boost::archive::text_oarchive oa(os);
        oa << MPsData;
        msg.DATA = os.str();
        os.clear();
        MPsData.clear();

        std::ostringstream oos;
        boost::archive::text_oarchive ooa(oos);
        ooa << transMs;
        msg.TRANSM = oos.str();
        oos.clear();

        insertMapPointsPub.publish(msg);

    }

    void PubToClient::pubUpdatedKFsToClients( std::set<LightKeyFrame> updatedKFs ) {

        corbslam_client::corbslam_message msg;

        std::vector< std::string > KFsData;
        KFsData.clear();

        for( std::set<LightKeyFrame>::iterator mit = updatedKFs.begin(); mit != updatedKFs.end(); mit++ ) {

            KeyFrame * tKF = (*mit).getKeyFrame();
            if( tKF ) {
                KeyFramePose* tKFP = new KeyFramePose( tKF );
                std::ostringstream os;
                boost::archive::text_oarchive oa(os);
                oa << tKFP ;
                KFsData.push_back( os.str() );
                os.clear();
            }
        }

        std::ostringstream os;
        boost::archive::text_oarchive oa(os);
        oa << KFsData;
        msg.DATA = os.str();
        os.clear();

        std::ostringstream oos;
        boost::archive::text_oarchive ooa(oos);
        ooa << transMs;
        msg.TRANSM = oos.str();
        oos.clear();

        updateKeyFramePosesPub.publish(msg);

    }

    void PubToClient::pubUpdatedMPSToClients( std::set<LightMapPoint> updatedMPs ) {

        corbslam_client::corbslam_message msg;

        std::vector< std::string > MPsData;
        MPsData.clear();

        for( std::set<LightMapPoint>::iterator mit = updatedMPs.begin(); mit != updatedMPs.end(); mit++ ) {
            MapPoint *tMP = (*mit).getMapPoint();
            if( tMP ) {
                MapPointPose * tMPP = new MapPointPose( tMP );
                std::ostringstream os;
                boost::archive::text_oarchive oa(os);
                oa << tMPP;
                std::string ss = os.str();
                MPsData.push_back(ss);
                os.clear();
                ss.clear();
            }
        }
        std::ostringstream os;
        boost::archive::text_oarchive oa(os);
        oa << MPsData;

        msg.DATA = os.str();
        os.clear();

        std::ostringstream oos;
        boost::archive::text_oarchive ooa(oos);
        ooa << transMs;
        msg.TRANSM = oos.str();
        oos.clear();

        updateMapPointPosesPub.publish(msg);

    }
}