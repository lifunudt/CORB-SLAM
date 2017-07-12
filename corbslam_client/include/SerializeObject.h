//
// Created by lifu on 17-2-7.
//

#ifndef ORB_SLAM2_SERIALIZEOBJECT_H
#define ORB_SLAM2_SERIALIZEOBJECT_H

#include <opencv2/opencv.hpp>
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/detail/basic_oserializer.hpp>


//#include "MapPoint.h"
//#include "Map.h"
//#include "LightMapPoint.h"
//#include "LightKeyFrame.h"

using namespace cv;


namespace boost {
    namespace serialization {

        //serialize the opencv mat class
        template<class Archive>
        void serialize(Archive &ar, cv::Mat &mat, const unsigned int) {
            int cols, rows, type;
            bool continuous;

            if (Archive::is_saving::value) {
                cols = mat.cols;
                rows = mat.rows;
                type = mat.type();
                continuous = mat.isContinuous();
            }

            ar & cols & rows & type & continuous;

            if (Archive::is_loading::value)
                mat.create(rows, cols, type);

            if (continuous) {
                const unsigned int data_size = rows * cols * mat.elemSize();
                ar & boost::serialization::make_array(mat.ptr(), data_size);
            }
            else {
                const unsigned int row_size = cols * mat.elemSize();
                for (int i = 0; i < rows; i++) {
                    ar & boost::serialization::make_array(mat.ptr(i), row_size);
                }
            }
        }

        // serialize KeyPoint class
        template<class Archive>
        void serialize(Archive &ar, cv::KeyPoint &kp, const unsigned int) {

            ar & kp.pt.x & kp.pt.y;
            ar & kp.angle & kp.size & kp.response & kp.octave & kp.class_id;

        }

        //serialize WordId class
        template<class Archive>
        void serialize(Archive &ar, DBoW2::WordId &Wid, const unsigned int) {
            ar & Wid;
        }

        //serialize WordValue class
        template<class Archive>
        void serialize(Archive &ar, DBoW2::WordValue &Wue, const unsigned int) {
            ar & Wue;
        }

        //serialize BowVector class
        //TODO : need to change the expression
        template<class Archive>
        void serialize(Archive &ar, DBoW2::BowVector &BVR, const unsigned int) {
            if (Archive::is_saving::value) {
                std::map<int, double> tBVR;
                for (DBoW2::BowVector::iterator mit = BVR.begin(); mit != BVR.end(); mit++) {
                    tBVR[mit->first] = mit->second;
                }
                ar & tBVR;
            } else if (Archive::is_loading::value) {
                std::map<int, double> tBVR;
                ar & tBVR;
                for (std::map<int, double>::iterator mit = tBVR.begin(); mit != tBVR.end(); mit++) {
                    BVR.addWeight(mit->first, mit->second);
                }
            }

        }

        //serialize FeatureVector class
        template<class Archive>
        void serialize(Archive &ar, DBoW2::FeatureVector &FVR, const unsigned int) {
            if (Archive::is_saving::value) {
                std::map<int, std::vector<unsigned int> > tFVR;
                for (DBoW2::FeatureVector::iterator mit = FVR.begin(); mit != FVR.end(); mit++) {
                    tFVR[mit->first] = mit->second;
                }
                ar & tFVR;
            } else if (Archive::is_loading::value) {
                std::map<int, std::vector<unsigned int>> tBVR;
                ar & tBVR;
                for (std::map<int, std::vector<unsigned int>>::iterator mit = tBVR.begin(); mit != tBVR.end(); mit++) {
                    FVR[mit->first] = mit->second;
                }
            }
        }


    }
}


#endif //ORB_SLAM2_SERIALIZEOBJECT_H
