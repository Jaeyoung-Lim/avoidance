#include "global_planner/motionprimitive_planner.h"

namespace global_planner {

MotionPrimitivePlanner::MotionPrimitivePlanner(const ros::NodeHandle& nh)
    : nh_(nh) {
  nh_.param<int>("num_primitives", num_primitives_, 6);
}

MotionPrimitivePlanner::~MotionPrimitivePlanner(){
    
}

void MotionPrimitivePlanner::updateFullOctomap(octomap::OcTree* octomap_world) {
  if (octomap_world_) delete octomap_world_;
  octomap_world_ = octomap_world;

}

bool MotionPrimitivePlanner::isTrajectoryCollisionFree(
    std::vector<Eigen::Vector3d> trajectory) {
  for (size_t i = 0; i < trajectory.size(); i++) {
    if (isPositionCollisionFree(trajectory[i])) return false;
  }
  return true;
}

bool MotionPrimitivePlanner::isPositionCollisionFree(Eigen::Vector3d position) {
  double occprob;
  int octree_depth = 16;

  octomap::OcTreeNode* node =
      octomap_world_->search(position(0), position(1), position(2), octree_depth);

  return occprob < 0.5;
}
}