#include "local_planner/local_planner.h"
#include "local_planner/common.h"
#include "local_planner/local_planner_node.h"
#include "local_planner/waypoint_generator.h"

#include <boost/algorithm/string.hpp>

int main(int argc, char** argv) {
  using namespace avoidance;
  ros::init(argc, argv, "local_planner_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_private("");

  LocalPlannerNode Node(nh, nh_private, true);
  ros::Duration(2).sleep();

  std::thread worker(&LocalPlannerNode::threadFunction, &Node);

  worker.join();
  return 0;
}
