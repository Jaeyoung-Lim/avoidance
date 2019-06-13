#include "global_planner/motionprimitive_planner.h"

namespace global_planner {

MotionPrimitivePlanner::MotionPrimitivePlanner(const ros::NodeHandle& nh) :
  nh_(nh),
  planning_horizon_(1.0),
  default_speed_(5.0),
  max_omega_(5.0),
  max_climbrate_(5.0) {

  nh_.param<int>("num_primitives", num_primitives_x, 3);
  nh_.param<int>("num_primitives", num_primitives_z, 3);
  num_primitives_ = num_primitives_z * num_primitives_x;

  GeneratePrimitives(curr_pos_);
  for(int i = 0;  i < num_primitives_; i++){
    primitivePub_.push_back(nh_.advertise<nav_msgs::Path>("/trajectory_publisher/primitiveset" + std::to_string(i), 1));
  }
  frame_id_ = "world";
  time_resolution_ = 0.1;
  map_initialized = false;
}

MotionPrimitivePlanner::~MotionPrimitivePlanner(){
    
}

void MotionPrimitivePlanner::updateFullOctomap(octomap::OcTree* octomap_world) {
  octomap_world_ = octomap_world;

  map_initialized = true;
}

void MotionPrimitivePlanner::GeneratePrimitives(Eigen::Vector3d start_position){
  motion_primitives_.resize(num_primitives_);

  for(size_t j = 0; j < num_primitives_z; j++){
    for(size_t i = 0; i < num_primitives_x; i++){
      //Set Current states
      int index = num_primitives_x * i + j;
      motion_primitives_[index].position = start_position;
      //Generate motion primitives from default_speed and omega
      motion_primitives_[index].velocity = default_speed_;

      if (i == double(num_primitives_x- 1) / 2) motion_primitives_[index].omega = 0.0;
      else motion_primitives_[index].omega = max_omega_ * (double(i+1) -  2) / ( double(num_primitives_x) );

      if (j == double(num_primitives_z- 1) / 2) motion_primitives_[index].climbrate = 0.0;
      else motion_primitives_[index].climbrate = max_climbrate_ * (double(j+1) -  2) / ( double(num_primitives_z) );
    }
  }  
}

void MotionPrimitivePlanner::PublishPrimitives()
{
  for(size_t i = 0; i < num_primitives_; i ++){
    nav_msgs::Path msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    double velocity = motion_primitives_[i].velocity;
    double omega = motion_primitives_[i].omega;
    double theta = yaw_;
    double vz = motion_primitives_[i].climbrate;

    if(!motion_primitives_[i].valid) continue;

    for(double t = 0; t < motion_primitives_[i].time_duration ; t+=time_resolution_){
      geometry_msgs::PoseStamped state;
      
      state.header.frame_id = frame_id_;
      state.header.stamp = ros::Time::now();
      state.pose.orientation.w = 1.0;
      state.pose.orientation.w = 0.0;
      state.pose.orientation.w = 0.0;
      state.pose.orientation.w = 0.0;
      if(omega == 0.0){
        state.pose.position.x = curr_pos_(0) + velocity *cos(theta)* time_resolution_* t; 
        state.pose.position.y = curr_pos_(1) + velocity *sin(theta)* time_resolution_* t; 
        state.pose.position.z = curr_pos_(2) + vz * time_resolution_* t;

      }else {
        state.pose.position.x = curr_pos_(0) + velocity / omega * (std::sin(omega * time_resolution_* t + theta) - std::sin(theta));
        state.pose.position.y = curr_pos_(1) + velocity / omega * (std::cos(theta) - std::cos(omega * time_resolution_* t + theta)); 
        state.pose.position.z = curr_pos_(2) + vz * time_resolution_* t; 

      }
      msg.poses.push_back(state);
    }
    primitivePub_[i].publish(msg);
  }
}

void MotionPrimitivePlanner::setGlobalPath(std::vector<geometry_msgs::PoseStamped> &path){
  //Convert global path into a local trajectory
  // if(path[0].poses.position ){

  // }
}

void MotionPrimitivePlanner::setInitialState(Eigen::Vector3d position){
  curr_pos_ = position;
  

}

void MotionPrimitivePlanner::GetOptimalPath(){
  GeneratePrimitives(curr_pos_);

  FindOptimalPrimitive();
  PublishPrimitives();
}

void MotionPrimitivePlanner::FindOptimalPrimitive(){
  int optimal_primitive_;
  double best_cost = 100;
  for(size_t i = 0; i < num_primitives_; i++){
    EvaluatePrimitive(motion_primitives_[i]);

    if(motion_primitives_[i].cost < best_cost){
      best_cost = motion_primitives_[i].cost;
      optimal_primitive_ = i;
    }
  }
}

void MotionPrimitivePlanner::EvaluatePrimitive(MotionPrimitive &primitive){
      
    double velocity = primitive.velocity;
    double omega = primitive.omega;
    double theta = yaw_;
    double vz = primitive.climbrate;

    std::vector<Eigen::Vector3d> trajectory;
    for(double t = 0; t < primitive.time_duration ; t+=time_resolution_){
    Eigen::Vector3d position;
      
      if(omega == 0.0){
        position(0) = curr_pos_(0) + velocity *cos(theta)* time_resolution_* t; 
        position(1) = curr_pos_(1) + velocity *sin(theta)* time_resolution_* t; 
        position(2) = curr_pos_(2) + vz * time_resolution_* t;

      }else {
        position(0) = curr_pos_(0) + velocity / omega * (std::sin(omega * time_resolution_* t + theta) - std::sin(theta));
        position(1) = curr_pos_(1) + velocity / omega * (std::cos(theta) - std::cos(omega * time_resolution_* t + theta)); 
        position(2) = curr_pos_(2) + vz * time_resolution_* t; 

      }
      trajectory.push_back(position);
    }
    
    //Mark primitive as invalid if obstacle is in collision
    primitive.valid = isTrajectoryCollisionFree(trajectory);
    //Calculate goal cost
    primitive.cost = calcGoalCost(trajectory);
}

double MotionPrimitivePlanner::calcGoalCost(std::vector<Eigen::Vector3d> trajectory){
  double distance = (trajectory.back() - goal_pos_).norm();

  return distance;
}

bool MotionPrimitivePlanner::isTrajectoryCollisionFree(
    std::vector<Eigen::Vector3d> trajectory) {
  for (size_t i = 0; i < trajectory.size(); i++) {
    if (!isPositionCollisionFree(trajectory[i])) return false;
  }
  return true;
}

bool MotionPrimitivePlanner::isPositionCollisionFree(Eigen::Vector3d position) {
  int octree_depth = 16;
  double occprob = 1.0;
  if(!map_initialized) return false;
  octomap::OcTreeNode* node =
      octomap_world_->search(position(0), position(1), position(2), octree_depth);

  if(node){
    occprob = node->getOccupancy();
  }

  return occprob < 0.5;
}
}