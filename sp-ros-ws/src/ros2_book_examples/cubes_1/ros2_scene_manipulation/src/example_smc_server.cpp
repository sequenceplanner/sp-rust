/*
#include <ros/ros.h>

#include <unordered_map>

#include "scene_manipulation_service.h"
#include "read_transforms_param.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "SceneManipulationService");

	ros::NodeHandle node;

	std::unordered_map<std::string, RelatedTransform> active_transforms = readTransformsParam(node, "/active_transforms");
	std::unordered_map<std::string, RelatedTransform> static_transforms = readTransformsParam(node, "/static_transforms");
	SceneManipulationService scene_manipulation_service(node, active_transforms, static_transforms);
	
	ROS_INFO("SceneManipulationService running");

	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();
	return 0;
};

*/
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_client");



  return 0;
}