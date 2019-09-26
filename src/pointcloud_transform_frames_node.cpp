/**
 * @author Luca Bartolomei, V4RL
 * @date   26.09.2019
 */
 
#include "pointcloud_utils/pointcloud_transform_frames.h"

int main(int argc, char** argv) {

  // Initialize ROS, start node.
  ros::init(argc, argv, "pointcloud_transform_frames_node");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  pointcloud_transform_frames::PointcloudTransformFrames pcl_transform_node(
        nh, nh_private);
 
  ros::spin();
  return 0;
}
