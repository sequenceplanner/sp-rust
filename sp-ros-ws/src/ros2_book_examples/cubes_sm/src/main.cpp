// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <inttypes.h>
#include <memory>
#include <unordered_map>
#include <iostream>
#include <fstream>

#include "ros2_scene_manipulation_msgs/srv/manipulate_scene.hpp"
#include "rclcpp/rclcpp.hpp"

#include "read_transforms.h"
#include "scene_manipulation_service.h"

using ManipulateScene = ros2_scene_manipulation_msgs::srv::ManipulateScene; 

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("scene_manipulation_node");

  std::ifstream t("/home/endre/eloquent_ros2_ws/src/ros2_book_examples/cubes_sm/active_transforms.json");
  std::string str((std::istreambuf_iterator<char>(t)),
                  std::istreambuf_iterator<char>());
  t.close();

  std::cout << str;

  std::unordered_map<std::string, RelatedTransform> active_transforms = readTransforms(str);

	std::unordered_map<std::string, RelatedTransform> static_transforms = readTransforms(str);

	SceneManipulationService scene_manipulation_service(node, active_transforms, static_transforms);

  //auto server = node.create_service<AddTwoInts>("add_two_ints", handle_service);
  rclcpp::spin(node);
  rclcpp::shutdown();
  node = nullptr;
  return 0;
}