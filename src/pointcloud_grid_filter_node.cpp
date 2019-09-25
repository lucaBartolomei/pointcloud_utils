#include "pointcloud_utils/pointcloud_grid_filter.h"

int main(int argc, char** argv) {

  // Initialize ROS, start node.
  ros::init(argc, argv, "pointcloud_grid_filter_node");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  pointcloud_grid_filter::PointcloudGridFilter pointcloud_grid_filter_node(
      nh, nh_private);
 
  ros::spin();
  return 0;
}
