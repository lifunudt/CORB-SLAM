#ifndef ROS_VIEWER_H
#define ROS_VIEWER_H
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nodelet/nodelet.h>
#include <iostream>
#include <mutex>

namespace My_Viewer {

struct rawData{
  cv::Mat im;
  cv::Mat depth;
  cv::Mat mTcw;
  double timestamp;
};

class ros_viewer
{
public:
  ros_viewer(const std::string &strSettingPath);
  void addKfToQueue(const cv::Mat im, const cv::Mat depthmap, const double timestamp, const cv::Mat mTcw);
  void addUpdatedKF(const std::map<double, cv::Mat> kfposes);
  void updateFullPointCloud();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr createPointCloud(const rawData rawimg, int step=1);

  // Main function
  void Run();

  std::mutex mMutexROSViewer;
private:
  //Calibration matrix
  cv::Mat mK;
  cv::Mat mDistCoef;
  float fx_;
  float fy_;
  float cx_;
  float cy_;
  float invfx;
  float invfy;
  cv::Mat mMapx;
  cv::Mat mMapy;
  cv::Mat mK_new;

  std::vector<rawData> rawImages_queue; // temp raw images
  std::vector<rawData> rawImages; //global raw images to be used when a loop is closed

  ros::Publisher pub_pointCloud;
  ros::Publisher pub_pointCloudFull;
  ros::Publisher pub_pointCloudupdated;

  std::map<double, cv::Mat> updatedKFposes;
  bool mbNeedUpdateKFs;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullCloud;
};

} // namespace
#endif // ROS_VIEWER_H
