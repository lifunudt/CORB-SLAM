/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

//#include "ros_viewer.h"

using namespace std;

//My_Viewer::ros_viewer* ros_view;
//std::thread* mptViewer;
//tf::TransformBroadcaster* tfb_;

//void pub_camera(const cv::Mat mTcw1);

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

// the world coordinates in ORB-SLAM was set to be the first frame coordinates
cv::Mat coordinateTransform(cv::Mat mTcw)
{
    // rotate to world coordinates
    float rot[3][3] = {{0,-1,0},{0,0,-1},{1,0,0}};
    float trans[3]  = {0.,0.,0.5};
    cv::Mat mR1w = cv::Mat(3,3,CV_32F,rot);
    cv::Mat mtw1 = cv::Mat(3,1,CV_32F,trans);

    cv::Mat mRc1 = mTcw.rowRange(0,3).colRange(0,3);
    cv::Mat mtc1 = mTcw.rowRange(0,3).col(3);
    cv::Mat mt1c = -mRc1.t()*mtc1;
    cv::Mat mRcw = mRc1*mR1w;
    cv::Mat mtcw = -mRc1*mt1c - mRcw*mtw1;

    cv::Mat mTcwr = cv::Mat::eye(4,4,CV_32F);
    mRcw.copyTo(mTcwr.rowRange(0,3).colRange(0,3));
    mtcw.copyTo(mTcwr.rowRange(0,3).col(3));

    return mTcwr.clone();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_client");
    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    /// setup ros viewer, a new thread
//    ros_view = new My_Viewer::ros_viewer(argv[2]);
//    mptViewer = new thread(&My_Viewer::ros_viewer::Run,ros_view);
//    tfb_ = new tf::TransformBroadcaster();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    ofstream ff;
    ff.open( "trackingcost.txt");
    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        cv::Mat mTcw(4,4,CV_32F);

        mTcw = SLAM.TrackRGBD(imRGB,imD,tframe);

//        pub_camera(mTcw);
//
//        if (SLAM.mbNewKeyframe){
//            ros_view->addKfToQueue(imRGB,imD,tframe, coordinateTransform(mTcw));
//        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);

    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}


//void pub_camera(const cv::Mat mTcw1)
//{
//    cv::Mat mTcw = coordinateTransform(mTcw1);
//    tf::Matrix3x3 rot(mTcw.at<float>(0,0), mTcw.at<float>(0,1), mTcw.at<float>(0,2),
//                      mTcw.at<float>(1,0), mTcw.at<float>(1,1), mTcw.at<float>(1,2),
//                      mTcw.at<float>(2,0), mTcw.at<float>(2,1), mTcw.at<float>(2,2));
//    tf::Vector3 position(mTcw.at<float>(0,3), mTcw.at<float>(1,3), mTcw.at<float>(2,3));
//    tf::Transform camtf = tf::Transform(rot,
//                                        position);
//    tf::StampedTransform tf_stamped(camtf.inverse(),
//                                    ros::Time::now(),
//                                    "world", "camera_link");
//    tfb_->sendTransform(tf_stamped);
//
//}
