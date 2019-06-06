#ifndef GLOBAL_PLANNER_MOTION_PRIMITIVE_PLANNER_H
#define GLOBAL_PLANNER_MOTION_PRIMITIVE_PLANNER_H

#include <Eigen/Core>
#include "ros/ros.h"

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

namespace global_planner {

class MotionPrimitivePlanner {
 private:
  ros::NodeHandle nh_;

  Eigen::Vector3d curr_pos_;

  int num_primitives_;

  octomap::OcTree* octomap_world_;

  void updateFullOctomap(octomap::OcTree* octomap_world);
  bool isTrajectoryCollisionFree(std::vector<Eigen::Vector3d> trajectory);
  bool isPositionCollisionFree(Eigen::Vector3d position);

 public:
  MotionPrimitivePlanner(const ros::NodeHandle& nh);
  ~MotionPrimitivePlanner();
};
}

#endif