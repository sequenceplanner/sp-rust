## ros2_book_examples

All examples can be launched without the GUI controller and/or without RViz, just provide:

`gui:=False` and/or `rviz:=False`

1. Cube handover 1:
   
   `ros2 launch ros2_book_examples cubes.launch.py`

2. Cube handover 2:
   
    `ros2 launch ros2_book_examples cubes_planning.launch.py`

### cubes

    Resources:
    1.  2 x Mecademic meca 500 robot
    2.  2 x Robotiq 2f 85 gripper

    Description:
    Some description + no motion planning

### cubes planning

    Resources:
    1.  2 x Mecademic meca 500 robot
    2.  2 x Robotiq 2f 85 gripper

    Description:
    Some description + motion planning