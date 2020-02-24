## ros2_book_examples

All examples can be launched without the GUI controller and/or without RViz, just provide:

`gui:=False` and/or `rviz:=False`

1. cubes_1:
   
   `ros2 launch cubes_1_bringup cubes_1.launch.py`

2. cubes_2:
   
    `ros2 launch cubes_2_bringup cubes_1.launch.py`

### cubes_1

    Resources:
    1.  2 x Mecademic meca 500 robot
    2.  2 x Robotiq 2f 85 gripper

    Description:
    Some description + no motion planning

### cubes_2

    Resources:
    1.  2 x Mecademic meca 500 robot
    2.  2 x Robotiq 2f 85 gripper

    Description:
    Some description + motion planning