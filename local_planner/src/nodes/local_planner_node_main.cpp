#include "local_planner/local_planner_node.h"

using namespace avoidance;

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_planner_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_private("");

  LocalPlannerNode Node(nh, nh_private, true);
  Node.startNode();
  ros::Duration(2).sleep();

  std::thread worker(&LocalPlannerNode::threadFunction, &Node);

  worker.join();
  return 0;
}
