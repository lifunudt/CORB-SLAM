//
// Created by lifu on 17-2-1.
//

#ifndef ORB_SLAM2_LIGHTKEYFRAME_H
#define ORB_SLAM2_LIGHTKEYFRAME_H


#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "Map.h"

#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/detail/basic_oserializer.hpp>

/*
 *
 */

namespace ORB_SLAM2 {
    class Map;
    class Cache;
    class KeyFrame;

    class LightKeyFrame {
    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive &ar,  const unsigned int) {
            ar & mnId;
        }

    public:
        //constructure
        LightKeyFrame();
        LightKeyFrame(long unsigned int pId, Cache *pCache);

        LightKeyFrame( KeyFrame * pKF );

        // replement the < operate function
        bool operator < ( const LightKeyFrame & lkf ) const{
            return this->mnId < lkf.mnId;
        }

        bool operator == ( const LightKeyFrame & lkf ) const{
            return this->mnId == lkf.mnId;
        }

        void setCacher( Cache *pCacher ){

            mpCache = pCacher;

        }

        KeyFrame* getKeyFrame() const;

        KeyFrame* getKeyFrameInCache();

        // the original keyframe ID
        long unsigned int mnId;
        //the referense of the cache
        Cache *mpCache;

    };
}



#endif //ORB_SLAM2_LIGHTKEYFRAME_H
