//
// Created by lifu on 2/15/17.
//

#include "DataDriver.h"
#include "sstream"
#include "ros/ros.h"
#include "time.h"

using namespace std;

namespace ORB_SLAM2 {

    DataDriver::DataDriver(Cache *pCache) {

        this->mpCache = pCache;

    }

    void DataDriver::updateKeyFramePosesToServer( std::set<LightKeyFrame> tUpdateKFs ) {

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<corbslam_server::corbslam_update>("updateKeyFrameToMap");
        corbslam_server::corbslam_update srv;

        std::vector< std::string > KFsData;

        KFsData.clear();


        for( std::set<LightKeyFrame>::iterator mit = tUpdateKFs.begin(); mit != tUpdateKFs.end(); mit++ ) {

            KeyFrame * tKF = (*mit).getKeyFrame();

            if( tKF && !tKF->getFixed() ) {
                KeyFramePose* tKFP = new KeyFramePose( tKF );

                std::ostringstream os;

                boost::archive::text_oarchive oa(os);
                oa << tKFP ;

                std::string ss = os.str();

                KFsData.push_back( ss );

                os.clear();
                ss.clear();
            }

        }

        std::ostringstream os;

        boost::archive::text_oarchive oa(os);

        oa << KFsData;

        std::string ss = os.str();

        srv.request.CID = mpCache->pClientId;
        srv.request.TYPE = 1;
        srv.request.COUNT = (int)KFsData.size();
        srv.request.DATA = ss;

        if( client.call( srv )) {

            cout << "update Keyframe poses to server " << endl;

        } else {

            ROS_INFO( "error at update keyframe poses to server");

        }

    }

    void DataDriver::updateMapPointPosesToServer(  std::set<LightMapPoint> tUpdateMPs ) {

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<corbslam_server::corbslam_update>("updateMapPointToMap");
        corbslam_server::corbslam_update srv;

        std::vector< std::string > MPsData;

        MPsData.clear();

        for( std::set<LightMapPoint>::iterator mit = tUpdateMPs.begin(); mit != tUpdateMPs.end(); mit++ ) {

            MapPoint *tMP = (*mit).getMapPoint();

            if( tMP && !tMP->getFixed()) {

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

        std::string ss = os.str();

        srv.request.CID = mpCache->pClientId;
        srv.request.TYPE = 2;
        srv.request.COUNT = (int)MPsData.size();
        srv.request.DATA = ss;

        if( client.call( srv )) {

            cout << "update MapPoint poses to server " << endl;

        } else {

            ROS_INFO( "error at update mappoint poses to server");

        }

    }

    void DataDriver::insertNewKeyFramesToServer( std::set<LightKeyFrame> newInsertedKFs ) {

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<corbslam_server::corbslam_insert>("insertKeyFrameToMap");
        corbslam_server::corbslam_insert srv;

        std::vector< std::string > KFsData;

        KFsData.clear();

        for( std::set<LightKeyFrame>::iterator mit = newInsertedKFs.begin(); mit != newInsertedKFs.end(); mit++ ) {

            KeyFrame * tKF = (*mit).getKeyFrame();

            if( tKF && !tKF->getFixed() ) {
                std::ostringstream os;

                boost::archive::text_oarchive oa(os);
                oa << tKF;

                std::string ss = os.str();

                KFsData.push_back( ss );

                os.clear();
                ss.clear();
            }

        }


        std::ostringstream os;

        boost::archive::text_oarchive oa(os);

        oa << KFsData;

        std::string ss = os.str();

        srv.request.CID = mpCache->pClientId;
        srv.request.TYPE = 3;
        srv.request.COUNT = (int)KFsData.size();
        srv.request.DATA = ss;

        os.clear();
        ss.clear();
        KFsData.clear();

        if( client.call( srv )) {

            cout << "insert new keyframes to server " << endl;

        } else {

            ROS_INFO( "error at insert keyframes to server");

        }

    }

    void DataDriver::insertNewMapPointsToServer( std::set<LightMapPoint> newInsertedMPs ) {

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<corbslam_server::corbslam_insert>("insertMapPointToMap");
        corbslam_server::corbslam_insert srv;

        std::vector< std::string > MPsData;

        MPsData.clear();

        for( std::set<LightMapPoint>::iterator mit = newInsertedMPs.begin(); mit != newInsertedMPs.end(); mit++ ) {

            MapPoint *tMP = (*mit).getMapPoint();

            if( tMP && !tMP->getFixed() ) {

                std::ostringstream os;

                boost::archive::text_oarchive oa(os);

                oa << tMP;

                std::string ss = os.str();

                MPsData.push_back(ss);

                os.clear();
                ss.clear();
            }

        }
        cout << endl;
        std::ostringstream os;

        boost::archive::text_oarchive oa(os);

        oa << MPsData;

        std::string ss = os.str();

        srv.request.CID = mpCache->pClientId;
        srv.request.TYPE = 4;
        srv.request.COUNT = (int)MPsData.size();
        srv.request.DATA = ss;

        os.clear();
        ss.clear();
        MPsData.clear();

        cout << "srv data size : " << srv.request.DATA.size() << " mps size : " << srv.request.COUNT << endl;

        if( client.call( srv )) {

            cout << "insert new mappoints to server " << endl;

        } else {

            ROS_INFO( "error at insert mappoints to server");

        }

    }

}

