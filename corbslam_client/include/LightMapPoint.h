//
// Created by lifu on 17-2-2.
//

#ifndef ORB_SLAM2_LIGHTMAPPOINT_H
#define ORB_SLAM2_LIGHTMAPPOINT_H



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
    class MapPoint;

    class LightMapPoint {
    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive &ar,  const unsigned int) {
            ar & mnMapPointId;
        }

    public:
        //constructure
        LightMapPoint();
        LightMapPoint(long unsigned int pId, Cache *pCache);

        LightMapPoint( MapPoint * pMP );

        // replement the < operate function
        bool operator < ( const LightMapPoint & lMP ) const{
            return this->mnMapPointId < lMP.mnMapPointId;
        };

        bool operator == ( const LightMapPoint & lMP ) const{
            return this->mnMapPointId == lMP.mnMapPointId;
        };

        MapPoint* getMapPoint() const;

        void setCacher( Cache *pCacher) {

            mpCache = pCacher;

        }

        // the original MapPoint ID
        long unsigned int mnMapPointId;
        //the referense of the cache
        Cache *mpCache;

    };
}

#endif //ORB_SLAM2_LIGHTMAPPOINT_H
