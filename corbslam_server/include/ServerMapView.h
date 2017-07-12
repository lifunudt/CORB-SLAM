//
// Created by lifu on 6/15/17.
//

#ifndef PROJECT_SERVERMAPVIEW_H
#define PROJECT_SERVERMAPVIEW_H

#include "MapDrawer.h"

#include <mutex>

using namespace ORB_SLAM2;

namespace CORBSLAM_SERVER{

    class ServerMapView{

    public:
        ServerMapView( MapDrawer* pMapDrawer, const string &strSettingPath);

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

        void RequestFinish();

        void RequestStop();

        bool isFinished();

        bool isStopped();

        void Release();

    private:

        bool Stop();

        ORB_SLAM2::MapDrawer* mpMapDrawer;

        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::mutex mMutexStop;
    };
}

#endif //PROJECT_SERVERMAPVIEW_H
