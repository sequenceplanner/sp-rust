# door example

A door can revolve around a hinge

1. ros2 launch sp_dummy_door_bringup dummy_robot_bringup.launch.py
2. open rviz2
3. publish a cmd to open or close, for example:
    ros2 topic pub /dummy_door_sp_to_interfacer std_msgs/String '{'data' : 'open'}'

# two dummy manipulator example

Two robots share a common zone and could collide

TODO: an interfacer node between sp and this example will be added to define useful poses + maybe some item to be handed over or something...

1. ros2 launch sp_dummy_robot_bringup dummy_robot_bringup.launch.py
2. open rviz2
3. publish a pose for robot1 with (for example):
    ros2 topic pub /joint_states sensor_msgs/msg/JointState '{[name:single_rrbot_joint1, single_rrbot_joint2], position:[-1, -1.3]}'
3. publish a pose for robot2 with (for example):
    ros2 topic pub /joint_states2 sensor_msgs/msg/JointState '{[name:2single_rrbot_joint1, 2single_rrbot_joint2], position:[-1, -1.3]}'