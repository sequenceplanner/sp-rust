
#include <rclcpp/rclcpp.hpp>

#include "ros2_scene_manipulation_msgs/srv/manipulate_scene.hpp"

#include <geometry_msgs/msg/transform.hpp>

namespace {
	using d = double;
	geometry_msgs::msg::Transform T(d x, d y, d z, d rx = 0.0,d ry = 0.0,d rz = 0.0,d rw = 1.0)
	{
		geometry_msgs::msg::Transform T;
		T.translation.x = x;
		T.translation.y = y;
		T.translation.z = z;
		T.rotation.x = rx;
		T.rotation.y = ry;
		T.rotation.z = rz;
		T.rotation.w = rw;
		return T;
	}
} // anonymous namespace

int main(int argc, char** argv){

    rclcpp::init(argc, argv);

	using ManipulateScene = ros2_scene_manipulation_msgs::srv::ManipulateScene; 

    auto node = rclcpp::Node::make_shared("minimal_client");
    auto client = node->create_client<ManipulateScene>("scene_manipulation_service");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
        return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
    }

  rclcpp::sleep_for(std::chrono::milliseconds(5000));


    auto request = std::make_shared<ManipulateScene::Request>();

    request->frame_id = "wobj_ladderframe";
    request->parent_id = "agv";
    request->transform = T(1.0,0.0,0.0);

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "service call failed :(");
        return 1;
    }
    auto result = result_future.get();



  rclcpp::sleep_for(std::chrono::milliseconds(5000));



    request->frame_id = "f2";
    request->parent_id = "f1";
    request->transform = T(0.0,1.0,0.0);

     result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "service call failed :(");
        return 1;
    }
     result = result_future.get();

    request->frame_id = "f1";
    request->parent_id = "world";
    request->transform = T(0.0,1.0,0.0);


  rclcpp::sleep_for(std::chrono::milliseconds(5000));


    result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "service call failed :(");
        return 1;
    }
    result = result_future.get();

    request->frame_id = "f1";
    request->parent_id = "agv";
    request->same_position_in_world = false;
    request->transform = T(0.0,1.0,0.0);

  rclcpp::sleep_for(std::chrono::milliseconds(5000));

    result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "service call failed :(");
        return 1;
    }
    result = result_future.get();
    return 0;
}