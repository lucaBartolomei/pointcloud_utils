/**
 * @author Luca Bartolomei, V4RL
 * @brief  Node that listens to a Pointcloud2 topic and continuosly pusblish it
 * @date   18.09.2019
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

sensor_msgs::PointCloud2 pcl_;
bool has_pcl_ = false;

void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &pcl_msg) {
  ROS_INFO("[Pcl Repeater] Received pointcloud");
  pcl_ = *pcl_msg;
  has_pcl_ = true;
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "pointcloud_repeater_node");
  ros::NodeHandle nh("pointcloud_repeater_node");
  ros::NodeHandle nh_private("~");

  ros::Subscriber pcl_sub;
  ros::Publisher pcl_pub;
  pcl_sub = nh_private.subscribe("pointcloud", 1, pointcloudCallback);
  pcl_pub = nh_private.advertise<sensor_msgs::PointCloud2>("pointcloud_out", 1);

  double rate;
  if(!nh.getParam("rate", rate)) {
    ROS_WARN("[Pcl Repeater] Rate not specified. Using 1 Hz");
    rate = 1.0;
  }

  ros::Rate ros_rate(rate);
  while(ros::ok()) {
    if(has_pcl_) {
      // Update the header
      pcl_.header.stamp = ros::Time::now();
      pcl_.header.seq++;

      pcl_pub.publish(pcl_);
    }

    ros::spinOnce();
    ros_rate.sleep();
  }
  return 0;
}