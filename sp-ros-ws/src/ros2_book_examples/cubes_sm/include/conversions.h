#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl/time.h"

#include "related_transform.h"

inline geometry_msgs::msg::TransformStamped toMsg(const RelatedTransform& relatedTransform)
{
	geometry_msgs::msg::TransformStamped geom_stamped;


	auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
	geom_stamped.header.stamp = ros_clock->now(); 



	geom_stamped.header.frame_id = relatedTransform.parent_id;
	geom_stamped.child_frame_id = relatedTransform.frame_id;
	geom_stamped.transform = static_cast<geometry_msgs::msg::Transform>(relatedTransform.transform);
	
	return geom_stamped;
} 

#endif