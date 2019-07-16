#include "avoidance/rviz_world_loader.h"

using namespace avoidance;

int main(int argc, char** argv) {
  ros::init(argc, argv, "rviz_world_loader_node");
  ros::NodeHandle nh_private("~");

  WorldVisualizer visualizer(nh_private);

  return 0;
}