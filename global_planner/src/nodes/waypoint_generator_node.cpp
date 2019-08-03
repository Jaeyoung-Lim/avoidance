#include "global_planner/waypoint_generator_node.hpp"

#include "avoidance/common.h"
#include "tf/transform_datatypes.h"

namespace avoidance {
const Eigen::Vector3f nan_setpoint = Eigen::Vector3f(NAN, NAN, NAN);

WaypointGeneratorNode::WaypointGeneratorNode(const ros::NodeHandle &nh) : nh_(nh), spin_dt_(0.1) {

  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped &>("/mavros/local_position/pose", 1,
                                                                &WaypointGeneratorNode::positionCallback, this);
  trajectory_sub_ = nh_.subscribe("/mavros/trajectory/desired", 1, &WaypointGeneratorNode::trajectoryCallback, this);
  mission_sub_ = nh_.subscribe("/mavros/mission/waypoints", 1, &WaypointGeneratorNode::missionCallback, this);
  state_sub_ = nh_.subscribe("/mavros/state", 1, &WaypointGeneratorNode::stateCallback, this);

  trajectory_pub_ = nh_.advertise<mavros_msgs::Trajectory>("/mavros/trajectory/generated", 10);
  land_hysteresis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/land_hysteresis", 1);
  marker_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("/goal_position", 1);

  waypointGenerator_.publishTrajectorySetpoints_ = [this](const Eigen::Vector3f &pos_sp, const Eigen::Vector3f &vel_sp,
                                                          float yaw_sp, float yaw_speed_sp) {
    publishTrajectorySetpoints(pos_sp, vel_sp, yaw_sp, yaw_speed_sp);
  };
}

void WaypointGeneratorNode::startNode() {
  ros::TimerOptions timer_options(ros::Duration(spin_dt_),
                                  boost::bind(&WaypointGeneratorNode::cmdLoopCallback, this, _1), &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(timer_options);
  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
}

void WaypointGeneratorNode::cmdLoopCallback(const ros::TimerEvent &event) {
  while (!grid_received_ && ros::ok()) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  waypointGenerator_.calculateWaypoint();
  landingAreaVisualization();
  goalVisualization();
  grid_received_ = false;

  return;
}

void WaypointGeneratorNode::positionCallback(const geometry_msgs::PoseStamped &msg) {
  waypointGenerator_.position_ = avoidance::toEigen(msg.pose.position);
  double roll, pitch, yaw;

  tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  tf::Matrix3x3 mat(q);
  mat.getRPY(roll, pitch, yaw);
  waypointGenerator_.yaw_ = static_cast<float>(yaw);
  ROS_INFO("[WGN] Current position %f %f %f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
}

void WaypointGeneratorNode::trajectoryCallback(const mavros_msgs::Trajectory &msg) {
  bool update = ((avoidance::toEigen(msg.point_2.position) - goal_visualization_).norm() > 0.01) ||
                waypointGenerator_.goal_.topRows<2>().array().hasNaN();

  if (update && msg.point_valid[0] == true) {
    waypointGenerator_.goal_ = avoidance::toEigen(msg.point_1.position);
    waypointGenerator_.velocity_setpoint_ = avoidance::toEigen(msg.point_1.velocity);

    ROS_INFO_STREAM("\033[1;33m [WGN] Set New goal from FCU " << waypointGenerator_.goal_.transpose()
                                                              << " - nan nan nan \033[0m");
  }
  if (msg.point_valid[1] == true) {
    goal_visualization_ = avoidance::toEigen(msg.point_2.position);
    waypointGenerator_.yaw_setpoint_ = msg.point_2.yaw;
    waypointGenerator_.yaw_speed_setpoint_ = msg.point_2.yaw_rate;
  }
}

void WaypointGeneratorNode::missionCallback(const mavros_msgs::WaypointList &msg) {
  waypointGenerator_.is_land_waypoint_ = false;
  for (auto waypoint : msg.waypoints) {
    if (waypoint.is_current && waypoint.command == 21) {
      waypointGenerator_.is_land_waypoint_ = true;
    }
  }
}

void WaypointGeneratorNode::stateCallback(const mavros_msgs::State &msg) {
  if (msg.mode == "AUTO.LAND") {
    waypointGenerator_.is_land_waypoint_ = true;
  } else if (msg.mode == "AUTO.MISSION") {
    // is_land_waypoint_ is set trought the mission item type
  } else {
    waypointGenerator_.is_land_waypoint_ = false;
    waypointGenerator_.trigger_reset_ = true;
  }

  if (msg.armed == false) {
    waypointGenerator_.is_land_waypoint_ = false;
    waypointGenerator_.trigger_reset_ = true;
  }
}

void WaypointGeneratorNode::publishTrajectorySetpoints(const Eigen::Vector3f &pos_sp, const Eigen::Vector3f &vel_sp,
                                                       float yaw_sp, float yaw_speed_sp) {
  mavros_msgs::Trajectory setpoint;
  setpoint.header.stamp = ros::Time::now();
  setpoint.header.frame_id = "local_origin";
  setpoint.type = 0;  // MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS
  setpoint.point_1.position.x = pos_sp.x();
  setpoint.point_1.position.y = pos_sp.y();
  setpoint.point_1.position.z = pos_sp.z();
  setpoint.point_1.velocity.x = vel_sp.x();
  setpoint.point_1.velocity.y = vel_sp.y();
  setpoint.point_1.velocity.z = vel_sp.z();
  setpoint.point_1.acceleration_or_force.x = NAN;
  setpoint.point_1.acceleration_or_force.y = NAN;
  setpoint.point_1.acceleration_or_force.z = NAN;
  setpoint.point_1.yaw = yaw_sp;
  setpoint.point_1.yaw_rate = yaw_speed_sp;

  fillUnusedTrajectorySetpoints(setpoint.point_2);
  fillUnusedTrajectorySetpoints(setpoint.point_3);
  fillUnusedTrajectorySetpoints(setpoint.point_4);
  fillUnusedTrajectorySetpoints(setpoint.point_5);

  setpoint.time_horizon = {NAN, NAN, NAN, NAN, NAN};

  bool xy_pos_sp_valid = std::isfinite(setpoint.point_1.position.x) && std::isfinite(setpoint.point_1.position.y);
  bool xy_vel_sp_valid = std::isfinite(setpoint.point_1.velocity.x) && std::isfinite(setpoint.point_1.velocity.y);

  if ((xy_pos_sp_valid || xy_vel_sp_valid) &&
      (std::isfinite(setpoint.point_1.position.z || std::isfinite(setpoint.point_1.velocity.z)))) {
    setpoint.point_valid = {true, false, false, false, false};
  } else {
    setpoint.point_valid = {false, false, false, false, false};
  }

  trajectory_pub_.publish(setpoint);
}

void WaypointGeneratorNode::fillUnusedTrajectorySetpoints(mavros_msgs::PositionTarget &point) {
  point.position.x = NAN;
  point.position.y = NAN;
  point.position.z = NAN;
  point.velocity.x = NAN;
  point.velocity.y = NAN;
  point.velocity.z = NAN;
  point.acceleration_or_force.x = NAN;
  point.acceleration_or_force.y = NAN;
  point.acceleration_or_force.z = NAN;
  point.yaw = NAN;
  point.yaw_rate = NAN;
}

int main(int argc, char **argv) {
  using namespace avoidance;
  ros::init(argc, argv, "waypoint_generator_node");
  ros::NodeHandle nh("~");

  WaypointGeneratorNode NodeWG(nh);
  NodeWG.startNode();
  while (ros::ok()) {
    ros::Duration(1.0).sleep();
  }

  return 0;
}
