# unification_ros2_messages

A ROS2 package providing ROS2 message types and mapping rules for bridging ROS1 and ROS2
messages for the unification project. Steps to add you own message types and succesfully bridge:

1. Define a message type in your ROS1 package (named for example: unification_roscontrol) and save it in the msg folder of the package. Name the message, for example messageX.msg.
2. Edit the CMakeLists.txt file of the ROS1 package:
    1. Uncomment the following section and add your message:
	```
	add_message_files(
  		FILES
		messageX.msg
	)
	```
    2. Uncomment line 5 in the catkin_pakcage section:
	```
	catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES unification_roscontrol
		#  CATKIN_DEPENDS other_catkin_pkg
		#  DEPENDS system_lib
    		CATKIN_DEPENDS message_runtime
	)
	```
    3. Uncomment the generate_messages section:
	```
	generate_messages(
		DEPENDENCIES
  		std_msgs  # Or other packages containing msgs
	)
	```
3. Edit the package.xml file so that you have the following lines uncommented:
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
<buildtool_depend>catkin</buildtool_depend>
```
4. Source your ROS1 environment file, cd into your workspace and catkin_make.
5. Define a message type in the unification_ros2_messages package (prefferably the same message type with the same name as in step 1) and save it in the msg folder of the unification_ros2_messages package.
6. Define a mapping rule in a messageX_mpr.yaml file between those two messages and save it in the unification_ros2_messages package folder. The mapping rule file should have the following format (NOTE: two spaces define one indentation):
```
-
  ros1_package_name: 'ros1_pkg_name'
  ros1_message_name: 'ros1_msg_name'
  ros2_package_name: 'ros2_pkg_name'
  ros2_message_name: 'ros2_msg_name'
    fields_1_to_2:
    ros1_msg_field_1: 'ros2_msg_field_1'
    ros1_msg_field_2: 'ros2_msg_field_2'
    ros1_msg_field_3: 'ros2_msg_field_3'
```	
7. Edit the CMakeLists.txt file of the ROS2 unification_ros2_messages package:
    1. Add your message type in the section:
	```
	rosidl_generate_interfaces(unification_ros2_messages
  		"msg/message1.msg"
  		"msg/message2.msg"
  		"msg/messageX.msg"
  		DEPENDENCIES builtin_interfaces
	)
	```
    2. After the BUILD_TESTING section, add the installation rule for the messageX_mpr.yaml file:
	```
	install(
		FILES messageX_mpr.yaml
  		DESTINATION share/${PROJECT_NAME})
	```
8. Edit the package.xml so that in the export section you add the mapping rule:
```
<export>
  <ros1_bridge mapping_rules="messageX_mpr.yaml/">
  <build_type>ament_cmake</build_type>
</export>

```
9. You can use the ros1_bridge's dynamic_bridge, but for a setup where you have nonconsistent publishing and subscribing, it is more robust to have one persistent static bridge per topic. To do this, cd intro the ros1_bridge's src folder and make a copy of the static_bridge.cpp and rename it as you like, for example: sb_topic_x.cpp. Open and edit:
```
// ROS 1 node
  ros::init(argc, argv, "ros1_sb_topic_x");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("ros2_sb_topic_x");

  // bridge one example topic
  std::string topic_name = "unification_roscontrol/topic_x";
  std::string ros1_type_name = "unification_roscontrol/messageX";
  std::string ros2_type_name = "unification_ros2_messages/messageX";
  size_t queue_size = 10;
```
10. Open the CMakeLists.txt of the ros1_bridge package and edit it so that you add executable generation of the .cpp you have made in the previos step like:
```
custom_executable(sb_topic_x
  "src/sb_topic_x.cpp"
  ROS1_DEPENDENCIES
  TARGET_DEPENDENCIES ${ros2_message_packages})
target_link_libraries(sb_topic_x
  ${PROJECT_NAME})
```
11. Delete the /build/ros1_bridge and the /install/ros1_bridge folders(not exactly sure why this is needed, but the ros1_bridge's factory generator gets confused if the bridge is not built clean. Otherwise it will also compile, but some remains of the previous build can cause problems).

12. In a fresh terminal, source your only your ROS2 environment, cd into the ROS2 workspace and build the workspace without the ros1_bridge with:
```
colcon build --symlink-install --packages-skip ros1_bridge
```
13. In another fresh terminal, source your ROS1 environment and then in the same terminal source the ROS2 environment. Now cd into your ROS2 workspace and build only the ros1_bridge with:
```
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```
14. All done. You should be able to have your bridge up and running when you:
    i. Open a terminal and source the ROS1 environment
    ii. Then, source the ROS2 environment
    iii. ros2 run ros1_bridge sb_topic_x

