#ifndef GLOBAL_PLANNER_MOTION_PRIMITIVE_PLANNER_H
#define GLOBAL_PLANNER_MOTION_PRIMITIVE_PLANNER_H

#include <Eigen/Core>
#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

namespace global_planner {

struct MotionPrimitive {
  double omega;
  double velocity;
  double time_duration = 10.0;

  Eigen::Vector3d position;
  double yaw;
};

class MotionPrimitivePlanner {
 private:
  ros::NodeHandle nh_;

  std::vector<ros::Publisher> primitivePub_;

  Eigen::Vector3d curr_pos_;

  int num_primitives_;
  double planning_horizon_;
  double default_speed_;
  double max_omega_;
  double time_resolution_;
  double yaw_;
  std::string frame_id_;

  octomap::OcTree* octomap_world_;
  std::vector<MotionPrimitive> motion_primitives_;

  void updateFullOctomap(octomap::OcTree* octomap_world);
  void GeneratePrimitives(Eigen::Vector3d);
  void PublishPrimitives();
  void setGlobalPath(std::vector<geometry_msgs::PoseStamped> &path);
  bool isTrajectoryCollisionFree(std::vector<Eigen::Vector3d> trajectory);
  bool isPositionCollisionFree(Eigen::Vector3d position);

 public:
  MotionPrimitivePlanner(const ros::NodeHandle& nh);
  ~MotionPrimitivePlanner();
  void GetOptimalPath();
  void setInitialState(Eigen::Vector3d position);

};
}

#endif