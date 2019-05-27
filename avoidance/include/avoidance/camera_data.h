#ifndef AVOIDANCE_CAMERA_DATA_H
#define AVOIDANCE_CAMERA_DATA_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include "ros/ros.h"
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud2.h"


namespace avoidance {

struct cameraData {
  std::string topic_;
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber camera_info_sub_;
  sensor_msgs::PointCloud2 newest_cloud_msg_;

  std::unique_ptr<std::mutex> trans_ready_mutex_;
  std::unique_ptr<std::condition_variable> trans_ready_cv_;

  std::unique_ptr<std::mutex> cloud_ready_mutex_;
  std::unique_ptr<std::condition_variable> cloud_ready_cv_;
  std::thread transform_thread_;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

  bool received_;
  bool transformed_;
};

class CameraData {
    public:
      CameraData(const ros::NodeHandle& nh);
      ~CameraData();

    /**
    * @brief     subscribes to all the camera topics and camera info
    * @param     camera_topics, array with the pointcloud topics strings
    **/
    void initializeCameraSubscribers(std::vector<std::string>& camera_topics);

    /**
    * @brief     computes the number of available pointclouds
    * @ returns  number of pointclouds
    **/
    size_t numReceivedClouds();

    /**
    * @brief     computes the number of transformed pointclouds
    * @ returns  number of transformed pointclouds
    **/
    size_t numTransformedClouds();

    /**
    * @brief     callaback for camera information
    * @param[in] msg, camera information message
    * @param[in] index, camera info instace number
    **/
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg,
                            int index);

    /**
    * @brief     callaback for pointcloud
    * @param[in] msg, pointcloud message
    * @param[in] index, pointcloud instance number
    **/
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg,
                            int index);
    std::vector<cameraData> cameras_;

    private:
      ros::NodeHandle nh_;

};
}

#endif  // AVOIDNACE_CAMERA_DATA_H
