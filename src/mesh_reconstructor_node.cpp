#include "pointcloud_utils/mesh_reconstructor.h"

int main(int argc, char** argv) {

  // Initialize ROS, start node.
  ros::init(argc, argv, "mesh_reconstructor_node");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  mesh_reconstructor::MeshReconstructor mesh_reconstructor_node(nh, nh_private);
 
  ros::spin();
  return 0;
}
