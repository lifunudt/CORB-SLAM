//
// Created by lifu on 2017/6/4.
//

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include <thread>

#include "MapFusion.h"
#include "ServerMap.h"

using namespace std;

using namespace CORBSLAM_SERVER;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Corbslam_server");
    ros::start();

    if( argc != 3) {
        cout << "lack of vocabulary path !!!\n";
        return false;
    }
    ROS_INFO("Corbslam_server start!");

    // Create MapFusion system. It initializes all system threads and gets ready to process frames.

    std::string strSettingPath = argv[2];

    MapFusion* mapfusion = new MapFusion( strSettingPath );

    mapfusion->loadORBVocabulary( argv[1] );

    mapfusion->createKeyFrameDatabase();

    ros::NodeHandle n;

    ros::ServiceServer InsertKeyFrameService = n.advertiseService("insertKeyFrameToMap", &MapFusion::insertKeyFrameToMap, mapfusion );

    ros::ServiceServer InsertMapPointService = n.advertiseService("insertMapPointToMap", &MapFusion::insertMapPointToMap, mapfusion);

    ros::ServiceServer updateKeyFrameToMapService = n.advertiseService("updateKeyFrameToMap", &MapFusion::updateKeyFrameToMap, mapfusion);

    ros::ServiceServer updateMapPointToMapService = n.advertiseService("updateMapPointToMap", &MapFusion::updateMapPointToMap, mapfusion);

    ROS_INFO("Publish services finished !");

    std::thread * mapFuisonThread = new thread(&MapFusion::fuseSubMapToMap, mapfusion);

    // publish update keyframe and mappoint poses topic
    std::thread * pubThread = new thread(&MapFusion::runPubTopic, mapfusion);

    // wait to get subcribe new keyframes or new mappoints
    ros::MultiThreadedSpinner spinner(2);

    spinner.spin();

    // Stop all threads

    // Save camera trajectory

    ros::shutdown();

    return 0;
}
