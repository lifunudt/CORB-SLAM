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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2 {

    long unsigned int MapPoint::nNextId = 1;
    mutex MapPoint::mGlobalMutex;

    MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Cache *pCacher) :
            mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<LightMapPoint>(nullptr)), mfMinDistance(0), mfMaxDistance(0), mpCacher(pCacher) {

        Pos.copyTo(mWorldPos);
        LightKeyFrame tLKF(pRefKF);
        mpRefKF = tLKF;
        mNormalVector = cv::Mat::zeros(3, 1, CV_32F);
        this->mnClientId = pCacher->pClientId;

        mTrackProjX = 0;
        mTrackProjY = 0;
        mTrackProjXR = 0;
        mbTrackInView = true;
        mnTrackScaleLevel = 0 ;
        mTrackViewCos = 0;

        ifFixed = false;

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpCacher->getMpMap()->mMutexPointCreation);

        if( nNextId == 1) nNextId = nNextId + (mpCacher->pClientId - 1 ) * 1000000;

        mnId = nNextId++;

    }

    MapPoint::MapPoint(const cv::Mat &Pos, Cache *pCacher, Frame *pFrame, const int &idxF) :
            mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
            mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<LightKeyFrame>(nullptr)), mnVisible(1),
            mnFound(1), mbBad(false), mpReplaced(nullptr), mpCacher(pCacher) {
        this->mnClientId = pCacher->pClientId;
        Pos.copyTo(mWorldPos);

        cv::Mat Ow = pFrame->GetCameraCenter();
        mNormalVector = mWorldPos - Ow;
        mNormalVector = mNormalVector / cv::norm(mNormalVector);

        cv::Mat PC = Pos - Ow;
        const float dist = cv::norm(PC);
        const int level = pFrame->mvKeysUn[idxF].octave;
        const float levelScaleFactor = pFrame->mvScaleFactors[level];
        const int nLevels = pFrame->mnScaleLevels;

        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

        pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

        mTrackProjX = 0;
        mTrackProjY = 0;
        mTrackProjXR = 0;
        mbTrackInView = true;
        mnTrackScaleLevel = 0 ;
        mTrackViewCos = 0;

        ifFixed = false;

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpCacher->getMpMap()->mMutexPointCreation);
        mnId = nNextId++;
    }

    void MapPoint::SetWorldPos(const cv::Mat &Pos) {
        //unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        Pos.copyTo(mWorldPos);
    }

    cv::Mat MapPoint::GetWorldPos() {
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos.clone();
    }

    cv::Mat MapPoint::GetNormal() {
        //unique_lock<mutex> lock(mMutexPos);
        return mNormalVector.clone();
    }

    void MapPoint::setFixed() {
        ifFixed = true;
    }

    bool MapPoint::getFixed() {
        return ifFixed;
    }

    LightKeyFrame MapPoint::GetLightReferenceKeyFrame() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mpRefKF;
    }

    KeyFrame *MapPoint::GetReferenceKeyFrame() {
        unique_lock<mutex> lock(mMutexFeatures);
        KeyFrame *tKF = mpRefKF.getKeyFrame();
        if (tKF != nullptr) {
            return tKF;
        } else {
            for (std::map<LightKeyFrame, size_t>::iterator mit = mObservations.begin();
                 mit != mObservations.end(); mit++) {
                if (mit->first.getKeyFrame()) {
                    mpRefKF = mit->first;
                    break;
                }

            }
            return mpRefKF.getKeyFrame();
        }

    }

    void MapPoint::AddObservation(KeyFrame *pKF, size_t idx) {

        if ((int)idx < (int)pKF->N) {
            unique_lock<mutex> lock(mMutexObservations);
            LightKeyFrame tLKF(pKF);
            if (mObservations.count(tLKF))
                return;
            mObservations[tLKF] = idx;

            if (pKF->mvuRight[idx] >= 0)
                nObs += 2;
            else
                nObs++;
        } else {

            cout << "add observation out of range\n";

            return;
        }

    }

    Cache *MapPoint::getCache() {
        return this->mpCacher;
    }

    void MapPoint::setCache( Cache * pCache ){

        this->mpCacher = pCache;
        this->mpRefKF.mpCache = pCache;
        std::map<LightKeyFrame, size_t> tObs;

        for (std::map<LightKeyFrame, size_t>::iterator mit = this->mObservations.begin();
             mit != this->mObservations.end(); mit++) {
            LightKeyFrame tLKF = mit->first;
            tLKF.mpCache = pCache;
            tObs[ tLKF ] = mit->second;
        }
        {
            unique_lock<mutex> lock(mMutexObservations);
            mObservations = tObs;
        }
        this->mpReplaced.mpCache = pCache;
    }


    void MapPoint::EraseObservation(KeyFrame *pKF) {
        bool bBad = false;
        {
            unique_lock<mutex> lock(mMutexObservations);
            LightKeyFrame tLKF(pKF);
            if (mObservations.find(tLKF) != mObservations.end()) {
                int idx = mObservations[tLKF];
                if (pKF->mvuRight[idx] >= 0)
                    nObs -= 2;
                else
                    nObs--;

                mObservations.erase(tLKF);

                if (mpRefKF == tLKF)
                    mpRefKF = mObservations.begin()->first;

                // If only 2 observations or less, discard point
                if (nObs <= 2)
                    bBad = true;
            }
        }

        if (bBad)
            SetBadFlag();
    }

    map<KeyFrame *, size_t> MapPoint::GetObservations() {
        unique_lock<mutex> lock(mMutexObservations);
        std::map<KeyFrame *, size_t> tmObs;
        for (std::map<LightKeyFrame, size_t>::iterator mit = mObservations.begin();
             mit != mObservations.end(); mit++) {
            if (this->mpCacher->KeyFrameInCache(mit->first.mnId)) {
                if ((mit->first).getKeyFrame())
                    tmObs[(mit->first).getKeyFrame()] = mit->second;
            }

        }
        return tmObs;
    }

    map<LightKeyFrame, size_t> MapPoint::GetLightObservations() {
        //unique_lock<mutex> lock(mMutexObservations);
        return mObservations;

    };

    std::vector<LightKeyFrame> MapPoint::GetLKFObeservations() {
        unique_lock<mutex> lock(mMutexObservations);
        std::vector<LightKeyFrame> tmObs;
        for (std::map<LightKeyFrame, size_t>::iterator mit = mObservations.begin();
             mit != mObservations.end(); mit++) {
            tmObs.push_back((mit->first));
        }
        return tmObs;

    }

    int MapPoint::Observations() {
        //unique_lock<mutex> lock(mMutexObservations);
        return nObs;
    }

    void MapPoint::SetBadFlag() {
        std::map<KeyFrame *, size_t> obs;
        {
            //unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            mbBad = true;
            obs = GetObservations();
            mObservations.clear();
        }
        for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
            KeyFrame *pKF = mit->first;
            pKF->EraseMapPointMatch(mit->second);
        }
        mpCacher->EraseMapPointFromMap(this);
    }

    MapPoint *MapPoint::GetReplaced() {
        unique_lock<mutex> lock1(mMutexFeatures);
        //unique_lock<mutex> lock2(mMutexPos);
        return mpReplaced.getMapPoint();
    }

    void MapPoint::Replace(MapPoint *pMP) {
        if (pMP->mnId == this->mnId)
            return;

        int nvisible, nfound;
        std::map<KeyFrame *, size_t> obs;
        {
            //unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            obs = GetObservations();
            mObservations.clear();
            mbBad = true;
            nvisible = mnVisible;
            mpReplaced = pMP;
        }
        {
            unique_lock<mutex> lock(mMutexFound);
            nfound = mnFound;
        }

        for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
            // Replace measurement in keyframe
            KeyFrame *pKF = mit->first;

            if (!pMP->IsInKeyFrame(pKF)) {
                pKF->ReplaceMapPointMatch(mit->second, pMP);
                pMP->AddObservation(pKF, mit->second);
            }
            else {
                pKF->EraseMapPointMatch(mit->second);
            }
        }
        pMP->IncreaseFound(nfound);
        pMP->IncreaseVisible(nvisible);
        pMP->ComputeDistinctiveDescriptors();

        mpCacher->EraseMapPointFromMap(this);
    }

    bool MapPoint::isBad() {
        //unique_lock<mutex> lock(mMutexFeatures);
        //unique_lock<mutex> lock2(mMutexPos);
        return mbBad;
    }

    void MapPoint::IncreaseVisible(int n) {
        unique_lock<mutex> lock(mMutexFeatures);
        mnVisible += n;
    }

    void MapPoint::IncreaseFound(int n) {
        unique_lock<mutex> lock(mMutexFound);
        mnFound += n;
    }

    float MapPoint::GetFoundRatio() {
        //unique_lock<mutex> lock(mMutexFound);
        return static_cast<float>(mnFound) / mnVisible;
    }

    void MapPoint::ComputeDistinctiveDescriptors() {
        // Retrieve all observed descriptors


        vector<cv::Mat> vDescriptors;

        std::map<KeyFrame *, size_t> observations;


        if (mbBad)
            return;

        observations = GetObservations();

        if (observations.empty())
            return;

        vDescriptors.reserve(observations.size());

        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {

            KeyFrame *pKF = mit->first;

            if (pKF && !pKF->isBad())
                vDescriptors.push_back(pKF->mDescriptors.row(mit->second));

        }

        if (vDescriptors.empty())
            return;

        // Compute distances between them
        const size_t N = vDescriptors.size();

        float Distances[N][N];
        for (size_t i = 0; i < N; i++) {
            Distances[i][i] = 0;
            for (size_t j = i + 1; j < N; j++) {
                int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
                Distances[i][j] = distij;
                Distances[j][i] = distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for (size_t i = 0; i < N; i++) {
            vector<int> vDists(Distances[i], Distances[i] + N);
            sort(vDists.begin(), vDists.end());
            int median = vDists[0.5 * (N - 1)];

            if (median < BestMedian) {
                BestMedian = median;
                BestIdx = i;
            }
        }

        {
            unique_lock<mutex> lock(mMutexFeatures);
            mDescriptor = vDescriptors[BestIdx].clone();

        }

    }

    cv::Mat MapPoint::GetDescriptor() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mDescriptor.clone();
    }

    int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        LightKeyFrame tLKF(pKF);
        if (mObservations.count(tLKF))
            return mObservations[tLKF];
        else
            return -1;
    }

    bool MapPoint::IsInKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        LightKeyFrame tLKF(pKF);
        return (mObservations.count(tLKF));
    }

    void MapPoint::UpdateNormalAndDepth() {
        std::map<KeyFrame *, size_t> observations;
        KeyFrame *pRefKF;
        cv::Mat Pos;
        {
            //unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            if (mbBad)
                return;
            Pos = mWorldPos.clone();
        }
        observations = GetObservations();
        pRefKF = GetReferenceKeyFrame();

        if (observations.empty() || pRefKF == nullptr)
            return;

        if ((int)observations[pRefKF] >= (int)pRefKF->N) {
            cout << "UpdateNormalAndDepth error pRefKF out of range \n ";
            return;
        }

        cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
        int n = 0;
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKF = mit->first;
            //cout << "second " << mit->second << endl;
            if (pKF) {
                cv::Mat Owi = pKF->GetCameraCenter();
                cv::Mat normali = mWorldPos - Owi;
                normal = normal + normali / cv::norm(normali);
                n++;
            }
        }

        cv::Mat PC = Pos - pRefKF->GetCameraCenter();
        const float dist = cv::norm(PC);
        const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
        const float levelScaleFactor = pRefKF->mvScaleFactors[level];
        const int nLevels = pRefKF->mnScaleLevels;

        {
            unique_lock<mutex> lock3(mMutexPos);
            mfMaxDistance = dist * levelScaleFactor;
            mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
            mNormalVector = normal / n;
        }
    }

    float MapPoint::GetMinDistanceInvariance() {
        unique_lock<mutex> lock(mMutexPos);
        return 0.8f * mfMinDistance;
    }

    float MapPoint::GetMaxDistanceInvariance() {
        unique_lock<mutex> lock(mMutexPos);
        return 1.2f * mfMaxDistance;
    }

    int MapPoint::PredictScale(const float &currentDist, KeyFrame *pKF) {
        float ratio;
        {
            unique_lock<mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pKF->mnScaleLevels)
            nScale = pKF->mnScaleLevels - 1;

        return nScale;
    }

    int MapPoint::PredictScale(const float &currentDist, Frame *pF) {
        float ratio;
        {
            unique_lock<mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pF->mnScaleLevels)
            nScale = pF->mnScaleLevels - 1;

        return nScale;
    }

} //namespace ORB_SLAM
