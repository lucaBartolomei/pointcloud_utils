/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main class for reading a PLY file and publish it as pointcloud2 topic
 * @date   12.09.2019
 */
 
 #pragma once

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

class PlyReader {

public:
	/**
   * @brief Constructor of the class
   * @param[in] nh : ROS node handle
   * @param[in] nh_private : Private ROS node handle
   */
  PlyReader(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
	
	/**
   * @brief Destructor
   */
  ~PlyReader();

  /**
   * @brief Method to check if we have a valid file to process
   * @return True if we have a path, False otherwise
   */
  bool hasValidFile() const { return has_ply_file_ || has_collada_file_; }

  /**
   * @brief Callback to read the ply file
   * @param[in] req : request for the service
   * @param[out] res : response to the service call
   * @return True if the service was triggered successfully, False otherwise
   */
  bool readFileServiceCallback(
          std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  /**
   * @brief Callback to publish a collada as a marker
   * @param[in] req : request for the service
   * @param[out] res : response to the service call
   * @return True if the service was triggered successfully, False otherwise
   */
  bool publishColladaServiceCallback(
          std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	
protected:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	ros::Publisher pointcloud_pub_;
	ros::Publisher collada_pub_;

	ros::ServiceServer read_file_srv_;
	ros::ServiceServer publish_collada_srv_;

	std::string output_frame_;
	std::string output_topic_;
	std::string ply_path_;
	std::string collada_path_;
	double scale_collada_;
	bool has_ply_file_;
	bool has_collada_file_;
	
	double position_collada_x_;
	double position_collada_y_;
	double position_collada_z_;
  double rotation_collada_roll_;
  double rotation_collada_pitch_;
	double rotation_collada_yaw_;

}; // end class PlyReader
