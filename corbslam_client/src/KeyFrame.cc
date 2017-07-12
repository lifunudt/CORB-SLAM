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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>

namespace ORB_SLAM2 {

    long unsigned int KeyFrame::nNextId = 1;
    std::mutex mMutexNextId;

    KeyFrame::KeyFrame(Frame &F, Cache *pCacher) :
            mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
            mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
            mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
            mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0), mRelocScore(0),
            fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
            mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
            mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
            mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
            mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
            mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
            mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpCacher(pCacher), mbFirstConnection(true),
            mpParent(nullptr),
            mbNotErase(false),
            mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb / 2) {
        {
            std::unique_lock<mutex> lock( mMutexNextId );

            if( KeyFrame::nNextId == 1) KeyFrame::nNextId = KeyFrame::nNextId + (mpCacher->pClientId - 1 ) * 1000000;

            mnId = KeyFrame::nNextId++;

            mGrid.resize(mnGridCols);
            for (int i = 0; i < mnGridCols; i++) {
                mGrid[i].resize(mnGridRows);
                for (int j = 0; j < mnGridRows; j++)
                    mGrid[i][j] = F.mGrid[i][j];
            }
            ifFixed = false;
            SetPose(F.mTcw);
            mTcwGBA.create(4,4,CV_32F);
            mTcwBefGBA.create(4,4,CV_32F);
            mTcp.create(4,4,CV_32F);
            this->mnClientId = pCacher->pClientId;
        }


    }

    void KeyFrame::ComputeBoW() {
        if (mBowVec.empty() || mFeatVec.empty()) {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            // Feature vector associate features with nodes in the 4th level (from leaves up)
            // We assume the vocabulary tree has 6 levels, change the 4 otherwise
            mpCacher->getMpVocabulary()->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    void KeyFrame::SetPose(const cv::Mat &Tcw_) {
        unique_lock<mutex> lock(mMutexPose);
        Tcw_.copyTo(Tcw);
        cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
        cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
        cv::Mat Rwc = Rcw.t();
        Ow = -Rwc * tcw;

        Twc = cv::Mat::eye(4, 4, Tcw.type());
        Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
        Ow.copyTo(Twc.rowRange(0, 3).col(3));
        cv::Mat center = (cv::Mat_<float>(4, 1) << mHalfBaseline, 0, 0, 1);
        Cw = Twc * center;
    }

    cv::Mat KeyFrame::GetPose() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.clone();
    }

    cv::Mat KeyFrame::GetPoseInverse() {
        unique_lock<mutex> lock(mMutexPose);
        return Twc.clone();
    }

    cv::Mat KeyFrame::GetCameraCenter() {
        unique_lock<mutex> lock(mMutexPose);
        return Ow.clone();
    }

    cv::Mat KeyFrame::GetStereoCenter() {
        unique_lock<mutex> lock(mMutexPose);
        return Cw.clone();
    }


    cv::Mat KeyFrame::GetRotation() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rowRange(0, 3).colRange(0, 3).clone();
    }

    cv::Mat KeyFrame::GetTranslation() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rowRange(0, 3).col(3).clone();
    }

    void KeyFrame::setFixed() {
        ifFixed = true;
    }

    bool KeyFrame::getFixed() {
        return ifFixed;
    }

    void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight) {
        {
            unique_lock<mutex> lock(mMutexConnections);

            LightKeyFrame pLKf(pKF);

            if (!mConnectedKeyFrameWeights.count(pLKf))
                mConnectedKeyFrameWeights[pLKf] = weight;
            else if (mConnectedKeyFrameWeights[pLKf] != weight)
                mConnectedKeyFrameWeights[pLKf] = weight;
            else
                return;
        }

        UpdateBestCovisibles();
    }

    void KeyFrame::UpdateBestCovisibles() {
        unique_lock<mutex> lock(mMutexConnections);
        vector<pair<int, LightKeyFrame> > vPairs;
        vPairs.reserve(mConnectedKeyFrameWeights.size());
        for (map<LightKeyFrame, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
             mit != mend; mit++)
            vPairs.push_back(make_pair(mit->second, mit->first));

        sort(vPairs.begin(), vPairs.end());
        list<LightKeyFrame> lKFs;
        list<int> lWs;
        for (size_t i = 0, iend = vPairs.size(); i < iend; i++) {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        mvpOrderedConnectedKeyFrames = vector<LightKeyFrame>(lKFs.begin(), lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
    }

    set<KeyFrame *> KeyFrame::GetConnectedKeyFrames() {
        unique_lock<mutex> lock(mMutexConnections);
        set<KeyFrame *> s;
        for (map<LightKeyFrame, int>::iterator mit = mConnectedKeyFrameWeights.begin();
             mit != mConnectedKeyFrameWeights.end(); mit++) {
            LightKeyFrame tLKF = mit->first;
            if( this->mpCacher->KeyFrameInCache( tLKF.mnId ) ) {
                KeyFrame *tKF = tLKF.getKeyFrame();
                if (tKF)
                    s.insert(tKF);
            }
        }
        return s;
    }

    set<LightKeyFrame> KeyFrame::GetConnectedLightKeyFrames() {
        unique_lock<mutex> lock(mMutexConnections);
        set<LightKeyFrame> s;
        if (mConnectedKeyFrameWeights.size() > 0) {
            for (map<LightKeyFrame, int>::iterator mit = mConnectedKeyFrameWeights.begin();
                 mit != mConnectedKeyFrameWeights.end(); mit++) {
                LightKeyFrame tLKF = mit->first;
                s.insert(tLKF);
            }
            return s;
        }
        return s;
    }

    vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames() {
        unique_lock<mutex> lock(mMutexConnections);
        std::vector<KeyFrame *> tCovisibleKeyFrame;
        for (int mit = 0; mit < (int) mvpOrderedConnectedKeyFrames.size(); mit++) {
            LightKeyFrame tLKF = mvpOrderedConnectedKeyFrames[mit];
            if (this->mpCacher->KeyFrameInCache(tLKF.mnId)) {
                KeyFrame *tKF = tLKF.getKeyFrame();
                if (tKF)
                    tCovisibleKeyFrame.push_back(tKF);
            }
        }
        return tCovisibleKeyFrame;
    }

    vector<KeyFrame *> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
        unique_lock<mutex> lock(mMutexConnections);
        std::vector<KeyFrame *> tCovisibleKeyFrame;
        for (int mit = 0; mit < (int) mvpOrderedConnectedKeyFrames.size(); mit++) {
            LightKeyFrame tLKF = mvpOrderedConnectedKeyFrames[mit];
            if (this->mpCacher->KeyFrameInCache(tLKF.mnId)) {
                KeyFrame *tKF = tLKF.getKeyFrame();
                if (tKF)
                    tCovisibleKeyFrame.push_back(tKF);
            }

        }
        if ((int) tCovisibleKeyFrame.size() < N)
            return tCovisibleKeyFrame;
        else
            return vector<KeyFrame *>(tCovisibleKeyFrame.begin(), tCovisibleKeyFrame.begin() + N);

    }

    vector<KeyFrame *> KeyFrame::GetCovisiblesByWeight(const int &w) {
        unique_lock<mutex> lock(mMutexConnections);

        std::vector<KeyFrame *> tOrderedConnectedKeyFrames;
        for (int mit = 0; mit < (int) mvpOrderedConnectedKeyFrames.size(); mit++) {
            LightKeyFrame tLKF = mvpOrderedConnectedKeyFrames[mit];
            if (this->mpCacher->KeyFrameInCache(tLKF.mnId)) {
                KeyFrame *tKF = nullptr;
                tKF = tLKF.getKeyFrame();
                //if (tKF)
                tOrderedConnectedKeyFrames.push_back(tKF);
            } else {
                KeyFrame *tKF = nullptr;
                tOrderedConnectedKeyFrames.push_back(tKF);
            }
        }

        if (tOrderedConnectedKeyFrames.empty())
            return vector<KeyFrame *>();

        vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w,
                                               KeyFrame::weightComp);
        if (it == mvOrderedWeights.end())
            return vector<KeyFrame *>();
        else {
            int n = it - mvOrderedWeights.begin();
            return vector<KeyFrame *>(tOrderedConnectedKeyFrames.begin(), tOrderedConnectedKeyFrames.begin() + n);
        }
    }

    int KeyFrame::GetWeight(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexConnections);
        LightKeyFrame pLKF(pKF);
        if (mConnectedKeyFrameWeights.count(pLKF))
            return mConnectedKeyFrameWeights[pLKF];
        else
            return 0;
    }

    void KeyFrame::setCache(Cache *pCache) {

        mpCacher = pCache;

        for (std::vector<LightMapPoint>::iterator mit = mvpMapPoints.begin();
             mit != mvpMapPoints.end(); mit++)
            (*mit).setCacher(pCache);

        std::map<LightKeyFrame, int> tmConnec;

        for (std::map<LightKeyFrame, int>::iterator mit = mConnectedKeyFrameWeights.begin();
             mit != mConnectedKeyFrameWeights.end(); mit++) {
            LightKeyFrame tLKF((*mit).first.mnId, pCache);
            tmConnec[tLKF] = (*mit).second;
        }
        mConnectedKeyFrameWeights = tmConnec;

        for (std::vector<LightKeyFrame>::iterator mit = mvpOrderedConnectedKeyFrames.begin();
             mit != mvpOrderedConnectedKeyFrames.end(); mit++) {
            (*mit).setCacher(pCache);
        }

        mpParent.setCacher(pCache);

        std::set<LightKeyFrame> tmspChildrens;
        for (std::set<LightKeyFrame>::iterator mit = mspChildrens.begin();
             mit != mspChildrens.end(); mit++) {
            LightKeyFrame tLKF((*mit).mnId, pCache);
            tmspChildrens.insert(tLKF);
        }
        mspChildrens = tmspChildrens;

        std::set<LightKeyFrame> tmspLoopEdges;
        for (std::set<LightKeyFrame>::iterator mit = mspLoopEdges.begin();
             mit != mspLoopEdges.end(); mit++) {
            LightKeyFrame tLKF((*mit).mnId, pCache);
            tmspLoopEdges.insert(tLKF);
        }
        mspLoopEdges = tmspLoopEdges;

    }

    void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        LightMapPoint tLMP(pMP);
        mvpMapPoints[idx] = tLMP;
        this->mpCacher->AddMapPointToMap(pMP);
    }

    void KeyFrame::EraseMapPointMatch(const size_t &idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx] = static_cast<LightMapPoint>(NULL);

    }

    void KeyFrame::EraseMapPointMatch(MapPoint *pMP) {
        int idx = pMP->GetIndexInKeyFrame(this);
        if (idx >= 0)
            mvpMapPoints[idx] = static_cast<LightMapPoint>(NULL);
    }

    void KeyFrame::dropMapPointMatches() {
        for (int mit = 0; mit < (int) mvpMapPoints.size(); mit++) {
            MapPoint *tmp = mvpMapPoints[mit].getMapPoint();
            if (tmp) {
                tmp->EraseObservation(this);
            }
        }

    }


    void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP) {
        if (pMP) {
            LightMapPoint tLMP(pMP);
            mvpMapPoints[idx] = tLMP;
        }
    }

    set<MapPoint *> KeyFrame::GetMapPoints() {
        unique_lock<mutex> lock(mMutexFeatures);
        set<MapPoint *> s;
        for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
            if (!mvpMapPoints[i].getMapPoint())
                continue;
            MapPoint *pMP = mvpMapPoints[i].getMapPoint();
            if (!pMP->isBad())
                s.insert(pMP);
        }
        return s;
    }

    int KeyFrame::TrackedMapPoints(const int &minObs) {
        unique_lock<mutex> lock(mMutexFeatures);

        int nPoints = 0;
        const bool bCheckObs = minObs > 0;
        for (int i = 0; i < N; i++) {
            MapPoint *pMP = mvpMapPoints[i].getMapPoint();
            if (pMP) {
                if (!pMP->isBad()) {
                    if (bCheckObs) {
                        if (mvpMapPoints[i].getMapPoint()->Observations() >= minObs)
                            nPoints++;
                    } else
                        nPoints++;
                }
            }
        }
        //cout << "nPoints :" << nPoints << endl;
        return nPoints;
    }

    vector<MapPoint *> KeyFrame::GetMapPointMatches() {
        unique_lock<mutex> lock(mMutexFeatures);
        vector<MapPoint *> tmvps;
        for (int mit = 0; mit < (int) mvpMapPoints.size(); mit++) {
            tmvps.push_back(mvpMapPoints[mit].getMapPoint());
        }
        return tmvps;
    }

    MapPoint *KeyFrame::GetMapPoint(const size_t &idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints[idx].getMapPoint();
    }

    std::vector<LightMapPoint> KeyFrame::GetLightMapPointMatches() {

        return mvpMapPoints;

    }

    void KeyFrame::UpdateConnections() {

        map<LightKeyFrame, int> KFcounter;

        vector<LightMapPoint> vpMP;

        {
            //unique_lock<mutex> lockMPs(mMutexFeatures);
            vpMP = GetLightMapPointMatches();
        }

        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        for (vector<LightMapPoint>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++) {
            MapPoint *pMP = (*vit).getMapPoint();

            if (!pMP)
                continue;

            if (pMP->isBad())
                continue;

            map<LightKeyFrame, size_t> observations = pMP->GetLightObservations();

            for (map<LightKeyFrame, size_t>::iterator mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
                //TODO: if there shoud be replaced by the id
                if (mit->first.mnId == mnId)
                    continue;
                KFcounter[mit->first]++;
            }
        }

        // This should not happen
        if (KFcounter.empty())
            return;

        //If the counter is greater than threshold add connection
        //In case no keyframe counter is over threshold add the one with maximum counter
        int nmax = 0;
        LightKeyFrame pKFmax = static_cast<LightKeyFrame>(NULL);
        int th = 15;


        set<pair<int, LightKeyFrame> > vPairs_set;
        vPairs_set.clear();
        for (map<LightKeyFrame, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++) {
            if( this->mpCacher->KeyFrameInCache( mit->first.mnId )) {
                if (mit->second > nmax) {
                    nmax = mit->second;
                    pKFmax = mit->first;
                }
                if (mit->second >= th) {
                    vPairs_set.insert(make_pair(mit->second, mit->first));
                    LightKeyFrame tLKF = (mit->first);
                    KeyFrame *tKF = tLKF.getKeyFrame();
                    if (tKF)
                        tKF->AddConnection(this, mit->second);
                }
            }
        }

        if (vPairs_set.empty()) {
            vPairs_set.insert(make_pair(nmax, pKFmax));
            pKFmax.getKeyFrame()->AddConnection(this, nmax);
        }

        vector<pair<int, LightKeyFrame> > vPairs;
        vPairs.clear();
        std::copy(vPairs_set.begin(), vPairs_set.end(), std::back_inserter( vPairs ) );

        sort(vPairs.begin(), vPairs.end());

        list<LightKeyFrame> lKFs;
        list<int> lWs;
        for (size_t i = 0; i < vPairs.size(); i++) {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        {
            unique_lock<mutex> lockCon(mMutexConnections);

            // mspConnectedKeyFrames = spConnectedKeyFrames;

            mConnectedKeyFrameWeights = KFcounter;
            mvpOrderedConnectedKeyFrames = vector<LightKeyFrame>(lKFs.begin(), lKFs.end());
            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

            if (mbFirstConnection && mnId != 0) {
                mpParent = mvpOrderedConnectedKeyFrames.front();
                if (mpParent.getKeyFrame()) {
                    mpParent.getKeyFrame()->AddChild(this);
                    mbFirstConnection = false;
                }
            }

        }
    }

    void KeyFrame::AddChild(KeyFrame *pKF) {
        //unique_lock<mutex> lockCon(mMutexConnections);
        LightKeyFrame tLKF(pKF);
        mspChildrens.insert(tLKF);
    }

    void KeyFrame::EraseChild(KeyFrame *pKF) {
        //unique_lock<mutex> lockCon(mMutexConnections);
        LightKeyFrame tLKF(pKF);
        mspChildrens.erase(tLKF);
    }

    void KeyFrame::ChangeParent(KeyFrame *pKF) {
        //unique_lock<mutex> lockCon(mMutexConnections);
        LightKeyFrame tLKF(pKF);
        mpParent = tLKF;
        pKF->AddChild(this);
    }

    set<KeyFrame *> KeyFrame::GetChilds() {
        unique_lock<mutex> lockCon(mMutexConnections);
        std::set<KeyFrame *> tMspC;
        for (set<LightKeyFrame>::iterator mit = mspChildrens.begin();
             mit != mspChildrens.end(); mit++) {
            if ((*mit).getKeyFrame())
                tMspC.insert((*mit).getKeyFrame());
        }
        return tMspC;
    }

    KeyFrame *KeyFrame::GetParent() {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mpParent.getKeyFrameInCache();
    }

    bool KeyFrame::hasChild(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        LightKeyFrame tLKF(pKF);
        return mspChildrens.count(tLKF);
    }

    void KeyFrame::AddLoopEdge(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mbNotErase = true;
        LightKeyFrame tLKF(pKF);
        mspLoopEdges.insert(tLKF);
    }

    set<KeyFrame *> KeyFrame::GetLoopEdges() {
        unique_lock<mutex> lockCon(mMutexConnections);
        std::set<KeyFrame *> tMspC;
        for (set<LightKeyFrame>::iterator mit = mspLoopEdges.begin();
             mit != mspLoopEdges.end(); mit++) {
            if ((*mit).getKeyFrame())
                tMspC.insert((*mit).getKeyFrame());
        }
        return tMspC;
    }

    void KeyFrame::SetNotErase() {
        unique_lock<mutex> lock(mMutexConnections);
        mbNotErase = true;
    }

    void KeyFrame::SetErase() {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mspLoopEdges.empty()) {
                mbNotErase = false;
            }
        }

        if (mbToBeErased) {
            SetBadFlag();
        }
    }

    void KeyFrame::SetBadFlag() {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mnId == 0)
                return;
            else if (mbNotErase) {
                mbToBeErased = true;
                return;
            }
        }

        for (map<LightKeyFrame, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
             mit != mend; mit++)
            if ((mit->first).getKeyFrame())
                (mit->first).getKeyFrame()->EraseConnection(this);

        for (size_t i = 0; i < mvpMapPoints.size(); i++)
            if (mvpMapPoints[i].getMapPoint())
                mvpMapPoints[i].getMapPoint()->EraseObservation(this);
        {
            unique_lock<mutex> lock(mMutexConnections);
            unique_lock<mutex> lock1(mMutexFeatures);

            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();

            // Update Spanning Tree
            set<KeyFrame *> sParentCandidates;
            sParentCandidates.insert(mpParent.getKeyFrame());

            // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
            // Include that children as new parent candidate for the rest
            while (!mspChildrens.empty()) {
                bool bContinue = false;

                int max = -1;
                KeyFrame *pC;
                KeyFrame *pP;

                for (set<LightKeyFrame>::iterator sit = mspChildrens.begin(), send = mspChildrens.end();
                     sit != send; sit++) {
                    KeyFrame *pKF = (*sit).getKeyFrame();
                    if (!pKF)
                        continue;
                    if (pKF->isBad())
                        continue;

                    // Check if a parent candidate is connected to the keyframe
                    vector<KeyFrame *> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                    for (size_t i = 0, iend = vpConnected.size(); i < iend; i++) {
                        for (set<KeyFrame *>::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end();
                             spcit != spcend; spcit++) {
                            if (vpConnected[i] && (*spcit)) {
                                if (vpConnected[i]->mnId == (*spcit)->mnId) {
                                    int w = pKF->GetWeight(vpConnected[i]);
                                    if (w > max) {
                                        pC = pKF;
                                        pP = vpConnected[i];
                                        max = w;
                                        bContinue = true;
                                    }
                                }
                            }
                        }
                    }
                }

                if (bContinue) {
                    pC->ChangeParent(pP);
                    sParentCandidates.insert(pC);
                    LightKeyFrame tLKF(pC);
                    mspChildrens.erase(tLKF);
                } else
                    break;
            }

            // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
            if (!mspChildrens.empty() && mpParent.getKeyFrame()) {
                for (set<LightKeyFrame>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++) {

                    if ((*sit).getKeyFrame())
                        (*sit).getKeyFrame()->ChangeParent(mpParent.getKeyFrame());
                }

                mpParent.getKeyFrame()->EraseChild(this);
                mTcp = Tcw * mpParent.getKeyFrame()->GetPoseInverse();
                mbBad = true;
            }

        }

        mpCacher->EraseKeyFrameFromMap(this);
        mpCacher->EraseKeyFrameFromDB(this);
//    mpMap->EraseKeyFrame(this);
//    mpKeyFrameDB->erase(this);

    }


    bool KeyFrame::isBad() {
        //unique_lock<mutex> lock(mMutexConnections);
        return mbBad;
    }

    void KeyFrame::EraseConnection(KeyFrame *pKF) {
        bool bUpdate = false;
        {
            unique_lock<mutex> lock(mMutexConnections);
            LightKeyFrame tLKF(pKF);
            if (mConnectedKeyFrameWeights.find(tLKF) != mConnectedKeyFrameWeights.end()) {
                mConnectedKeyFrameWeights.erase(mConnectedKeyFrameWeights.find(tLKF));
                bUpdate = true;
            }
        }

        if (bUpdate)
            UpdateBestCovisibles();
    }

    vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0, (int) floor((x - mnMinX - r) * mfGridElementWidthInv));
        if (nMinCellX >= mnGridCols)
            return vIndices;

        const int nMaxCellX = min((int) mnGridCols - 1, (int) ceil((x - mnMinX + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int) floor((y - mnMinY - r) * mfGridElementHeightInv));
        if (nMinCellY >= mnGridRows)
            return vIndices;

        const int nMaxCellY = min((int) mnGridRows - 1, (int) ceil((y - mnMinY + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
                const vector<size_t> vCell = mGrid[ix][iy];
                for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool KeyFrame::IsInImage(const float &x, const float &y) const {
        return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
    }

    cv::Mat KeyFrame::UnprojectStereo(int i) {
        const float z = mvDepth[i];
        if (z > 0) {
            const float u = mvKeys[i].pt.x;
            const float v = mvKeys[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

            unique_lock<mutex> lock(mMutexPose);
            return Twc.rowRange(0, 3).colRange(0, 3) * x3Dc + Twc.rowRange(0, 3).col(3);
        } else
            return cv::Mat();
    }

    Cache *KeyFrame::getCache() {
        return mpCacher;
    }

    float KeyFrame::ComputeSceneMedianDepth(const int q) {
        vector<MapPoint *> vpMapPoints;
        cv::Mat Tcw_;
        {
            // unique_lock<mutex> lock(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPose);
            vpMapPoints = GetMapPointMatches();
            Tcw_ = Tcw.clone();
        }

        vector<float> vDepths;
        vDepths.reserve(N);
        cv::Mat Rcw2 = Tcw_.row(2).colRange(0, 3);
        Rcw2 = Rcw2.t();
        float zcw = Tcw_.at<float>(2, 3);
        for (int i = 0; i < N; i++) {
            if (mvpMapPoints[i].getMapPoint()) {
                MapPoint *pMP = mvpMapPoints[i].getMapPoint();
                cv::Mat x3Dw = pMP->GetWorldPos();
                float z = Rcw2.dot(x3Dw) + zcw;
                vDepths.push_back(z);
            }
        }

        sort(vDepths.begin(), vDepths.end());

        return vDepths[(vDepths.size() - 1) / q];
    }

} //namespace ORB_SLAM
