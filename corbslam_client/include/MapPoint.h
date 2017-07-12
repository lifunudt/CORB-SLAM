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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Cache.h"
#include "LightKeyFrame.h"
#include <opencv2/core/core.hpp>
#include "SerializeObject.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/set.hpp>
#include <mutex>

namespace ORB_SLAM2 {

    class KeyFrame;

    class Map;

    class Frame;

    class LightKeyFrame;

    class Cache;

    class MapPoint {
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & mnId & mnFirstKFid & mnFirstFrame & nObs;
            ar & mnClientId;
            ar & mTrackProjX & mTrackProjY & mTrackProjXR & mbTrackInView & mnTrackScaleLevel;
            ar & mTrackViewCos & mnTrackReferenceForFrame & mnLastFrameSeen;
            ar & mnBALocalForKF & mnFuseCandidateForKF;
            ar & mnLoopPointForKF & mnCorrectedByKF & mnCorrectedReference;
            ar & mPosGBA;
            ar & mnBAGlobalForKF;
            ar & mWorldPos;
            ar & mObservations;
            ar & mNormalVector;
            ar & mDescriptor;
            ar & mpRefKF;
            ar & mnVisible;
            ar & mnFound;
            ar & mbBad;
            ar & mpReplaced;
            ar & mfMinDistance;
            ar & mfMaxDistance;
        }

    public:
        MapPoint() {}

        MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Cache *pCacher);

        MapPoint(const cv::Mat &Pos, Cache *pCacher, Frame *pFrame, const int &idxF);

        void SetWorldPos(const cv::Mat &Pos);

        cv::Mat GetWorldPos();

        cv::Mat GetNormal();

        void setFixed();
        bool getFixed();

        KeyFrame *GetReferenceKeyFrame();

        LightKeyFrame GetLightReferenceKeyFrame();

        std::map<KeyFrame *, size_t> GetObservations();

        std::map<LightKeyFrame, size_t > GetLightObservations();

        std::vector<LightKeyFrame> GetLKFObeservations();

        int Observations();

        void AddObservation(KeyFrame *pKF, size_t idx);

        void EraseObservation(KeyFrame *pKF);

        int GetIndexInKeyFrame(KeyFrame *pKF);

        bool IsInKeyFrame(KeyFrame *pKF);

        void SetBadFlag();

        bool isBad();

        void Replace(MapPoint *pMP);

        MapPoint *GetReplaced();

        void IncreaseVisible(int n = 1);

        void IncreaseFound(int n = 1);

        float GetFoundRatio();

        inline int GetFound() {
            return mnFound;
        }

        void ComputeDistinctiveDescriptors();

        cv::Mat GetDescriptor();

        void UpdateNormalAndDepth();

        float GetMinDistanceInvariance();

        float GetMaxDistanceInvariance();

        int PredictScale(const float &currentDist, KeyFrame *pKF);

        int PredictScale(const float &currentDist, Frame *pF);

        Cache *getCache();

        void setCache( Cache * pCache );

    public:
        long unsigned int mnId;
        static long unsigned int nNextId;

        int mnClientId;
        long int mnFirstKFid;
        long int mnFirstFrame;
        int nObs;

        bool ifFixed;

        // Variables used by the tracking
        float mTrackProjX;
        float mTrackProjY;
        float mTrackProjXR;
        bool mbTrackInView;
        int mnTrackScaleLevel;
        float mTrackViewCos;
        long unsigned int mnTrackReferenceForFrame;
        long unsigned int mnLastFrameSeen;

        // Variables used by local mapping
        long unsigned int mnBALocalForKF;
        long unsigned int mnFuseCandidateForKF;

        // Variables used by loop closing
        long unsigned int mnLoopPointForKF;
        long unsigned int mnCorrectedByKF;
        long unsigned int mnCorrectedReference;
        cv::Mat mPosGBA;
        long unsigned int mnBAGlobalForKF;


        static std::mutex mGlobalMutex;

        // Keyframes observing the point and associated index in keyframe
        std::map<LightKeyFrame, size_t> mObservations;

    protected:


        // Position in absolute coordinates
        cv::Mat mWorldPos;

        // Mean viewing direction
        cv::Mat mNormalVector;

        // Best descriptor to fast matching
        cv::Mat mDescriptor;

        // Reference KeyFrame
        LightKeyFrame mpRefKF;

        // Tracking counters
        int mnVisible;
        int mnFound;

        // Bad flag (we do not currently erase MapPoint from memory)
        bool mbBad;
        LightMapPoint mpReplaced;

        // Scale invariance distances
        float mfMinDistance;
        float mfMaxDistance;

        Cache *mpCacher;

        std::mutex mMutexPos;
        std::mutex mMutexFeatures;
        std::mutex mMutexFound;
        std::mutex mMutexObservations;

    };

} //namespace ORB_SLAM

#endif // MAPPOINT_H
