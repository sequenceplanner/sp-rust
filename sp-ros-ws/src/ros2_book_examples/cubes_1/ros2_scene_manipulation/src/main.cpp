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
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "read_transforms.h"
#include "scene_manipulation_service.h"

using ManipulateScene = ros2_scene_manipulation_msgs::srv::ManipulateScene;

std::string readFile(const char* path)
{
    std::ifstream t(path);
    std::string file_contents((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());
    t.close();
    return file_contents;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::string package_share_directory = "";
    try {
        // may throw PackageNotFoundError exception
        package_share_directory = ament_index_cpp::get_package_share_directory("ros2_scene_manipulation");
    } catch(const std::exception& e) {
        fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }

    printf("package share: %s\n", package_share_directory.c_str());

    std::string active_fn = package_share_directory + "/active_transforms.json";
    std::string static_fn = package_share_directory + "/static_transforms.json";

    std::string active_transforms_path = readFile(active_fn.c_str());
    std::string static_transforms_path = readFile(static_fn.c_str());

    std::unordered_map<std::string, RelatedTransform> active_transforms = readTransforms(active_transforms_path);
    std::unordered_map<std::string, RelatedTransform> static_transforms = readTransforms(static_transforms_path);

    auto scene_manipulation_service = std::make_shared<SceneManipulationService>(active_transforms, static_transforms);

    rclcpp::spin(scene_manipulation_service);
    rclcpp::shutdown();
    return 0;
}
