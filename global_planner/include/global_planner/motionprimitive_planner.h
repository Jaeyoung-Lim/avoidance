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
  double climbrate;
  double time_duration = 10.0;

  bool valid;

  double cost = 0.0;

  Eigen::Vector3d position;
  double yaw;
};

class MotionPrimitivePlanner {
 private:
  ros::NodeHandle nh_;

  std::vector<ros::Publisher> primitivePub_;

  Eigen::Vector3d curr_pos_, curr_vel_;
  Eigen::Vector3d goal_pos_;

  int num_primitives_, num_primitives_x, num_primitives_z;
  int optimal_primitive_;
  double planning_horizon_;
  double default_speed_;
  double max_omega_;
  double max_climbrate_;
  double time_resolution_;
  double yaw_;
  bool map_initialized;
  std::string frame_id_;

  octomap::OcTree* octomap_world_;
  std::vector<MotionPrimitive> motion_primitives_;

  void GeneratePrimitives(Eigen::Vector3d);
  void PublishPrimitives();
  void setGlobalPath(std::vector<geometry_msgs::PoseStamped> &path);
  void FindOptimalPrimitive();
  void EvaluatePrimitive(MotionPrimitive &primitive);
  double calcGoalCost(std::vector<Eigen::Vector3d> trajectory);
  bool isTrajectoryCollisionFree(std::vector<Eigen::Vector3d> trajectory);
  bool isPositionCollisionFree(Eigen::Vector3d position);


 public:
  MotionPrimitivePlanner(const ros::NodeHandle& nh);
  ~MotionPrimitivePlanner();
  void GetOptimalPath();
  void updateFullOctomap(octomap::OcTree* octomap_world);
  void setInitialState(Eigen::Vector3d position, Eigen::Vector3d velocity);

};
}

#endif