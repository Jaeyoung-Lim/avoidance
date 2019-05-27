#include "avoidance/camera_data.h"

namespace avoidance{

CameraData::CameraData(const ros::NodeHandle& nh):
    nh_(nh) {
    
    std::vector<std::string> camera_topics;
    nh_.getParam("pointcloud_topics", camera_topics);
    initializeCameraSubscribers(camera_topics);
}

CameraData::~CameraData() {

}


void CameraData::initializeCameraSubscribers(
    std::vector<std::string>& camera_topics) {
  cameras_.resize(camera_topics.size());

  // create sting containing the topic with the camera info from
  // the pointcloud topic
  std::string s;
  s.reserve(50);
  std::vector<std::string> camera_info(camera_topics.size(), s);

  for (size_t i = 0; i < camera_topics.size(); i++) {
    cameras_[i].trans_ready_mutex_.reset(new std::mutex);
    cameras_[i].trans_ready_cv_.reset(new std::condition_variable);
    cameras_[i].cloud_ready_mutex_.reset(new std::mutex);
    cameras_[i].cloud_ready_cv_.reset(new std::condition_variable);
    cameras_[i].transformed_ = false;

    cameras_[i].pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        camera_topics[i], 1,
        boost::bind(&CameraData::pointCloudCallback, this, _1, i));
    cameras_[i].topic_ = camera_topics[i];
    cameras_[i].received_ = false;

    // get each namespace in the pointcloud topic and construct the camera_info
    // topic
    std::vector<std::string> name_space;
    boost::split(name_space, camera_topics[i], [](char c) { return c == '/'; });
    for (int k = 0, name_spaces = name_space.size() - 1; k < name_spaces; ++k) {
      camera_info[i].append(name_space[k]);
      camera_info[i].append("/");
    }
    camera_info[i].append("camera_info");
    cameras_[i].camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>(
        camera_info[i], 1,
        boost::bind(&CameraData::cameraInfoCallback, this, _1, i));
  }
}

size_t CameraData::numReceivedClouds() {
  size_t num_received_clouds = 0;
  for (size_t i = 0; i < cameras_.size(); i++) {
    if (cameras_[i].received_) num_received_clouds++;
  }
  return num_received_clouds;
}

size_t CameraData::numTransformedClouds() {
  size_t num_transformed_clouds = 0;
  for (size_t i = 0; i < cameras_.size(); i++) {
    std::unique_lock<std::mutex> lk(*(cameras_[i].trans_ready_mutex_));
    cameras_[i].trans_ready_cv_->wait_for(
        lk, std::chrono::milliseconds(30),
        [this, i] { return cameras_[i].transformed_; });
    if (cameras_[i].transformed_) num_transformed_clouds++;
  }
  return num_transformed_clouds;
}

void CameraData::pointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg, int index) {
  cameras_[index].newest_cloud_msg_ = *msg;  // FIXME: avoid a copy
  cameras_[index].received_ = true;

  {
    std::unique_lock<std::mutex> lck(*(cameras_[index].cloud_ready_mutex_));
    cameras_[index].transformed_ = false;
    cameras_[index].cloud_ready_cv_->notify_one();
  }
}

void CameraData::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg, int index) {
  // calculate the horizontal and vertical field of view from the image size and
  // focal length:
  // h_fov = 2 * atan (image_width / (2 * focal_length_x))
  // v_fov = 2 * atan (image_height / (2 * focal_length_y))
  // Assumption: if there are n cameras the total horizonal field of view is n
  // times the horizontal field of view of a single camera
  float h_fov = static_cast<float>(
      static_cast<double>(cameras_.size()) * 2.0 *
      atan(static_cast<double>(msg->width) / (2.0 * msg->K[0])) * 180.0 / M_PI);
  float v_fov = static_cast<float>(
      2.0 * atan(static_cast<double>(msg->height) / (2.0 * msg->K[4])) * 180.0 /
      M_PI);

//   local_planner_->setFOV(h_fov, v_fov);
//   wp_generator_->setFOV(h_fov, v_fov);
}

}