/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main executable that read the ply file and publishes it as pointcloud
 * @date   12.09.2019
 */

#include "pointcloud_utils/ply_reader.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "ply_reader_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create the Node
  PlyReader ply_reader(nh, nh_private);

  if(!ply_reader.hasValidFile()) {
    return -1;
  }

  ros::spin();
  return 0;
}
