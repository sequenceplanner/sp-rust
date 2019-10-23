# door example

A door can revolve around a hinge

1. ros2 launch sp_dummy_door_bringup dummy_robot_bringup.launch.py
2. open rviz2 (\door_description)
3. publish a cmd to open or close, for example:
    ros2 topic pub /dummy_door_sp_to_interfacer std_msgs/String '{'data' : 'open'}'

# two dummy manipulator example

Two robots share a common zone and could collide

1. ros2 launch sp_dummy_r1_r2_bringup dummy_robot_bringup.launch.py 
2. open rviz2 (\r1_description, \r2_description)
3. publish a pose for robot1 with (for example):
    ros2 topic pub /r1_sp_to_interfacer std_msgs/msg/String '{'data' : 'at'}'
4. publish a pose for robot2 with (for example):
    ros2 topic pub /r2_sp_to_interfacer std_msgs/msg/String '{'data' : 'at'}'