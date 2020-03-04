Cylinders example
====================

To run with actual driver
```
ros2 launch ros2_dorna_bringup robot_bringup_driver.launch.py rviz:=True gui:=True homing:=False
```
When the robot needs homing, pass `homing:=True` to the launch file as per above.

To run with simulator
```
ros2 launch ros2_dorna_bringup robot_bringup_simulator.launch.py rviz:=True gui:=True
```

The flags `gui:=False` and/or `rviz:=False` toggles starting of the gui and rviz windows.


TODO
--------------------
1. Gripper
2. Robot speed
3. Conveyor
4. SP Model


