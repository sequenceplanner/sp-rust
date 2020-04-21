from dorna import Dorna
import json
import time

# creat Dorna object and connect to the robot
robot = Dorna('./my_config_new.yaml')
robot.connect()


print(robot.homed())
print(robot.position())

print(robot.home("j0"))
print(robot.homed())
print(robot.position())
print(robot.home("j1"))
print(robot.homed())
print(robot.position())
print(robot.home("j2"))
print(robot.homed())
print(robot.position())
print(robot.home("j3"))
print(robot.homed())
print(robot.position())

# pose: pre shelf [-60, 120, -120, -10, 0]

# open servo robot.play({"command": "set_io", "prm": { "servo": 500 }})
# servo 0 fully opened


# pose: at shelf [-63, 85, -95, -10, 0]




#robot.calibrate([0, 0, 0, 0.0, 0.0])

#print(robot.position())

#robot.save_config('my_config_new.yaml')

# home all the joints
#robot.home(["j0", "j1", "j2", "j3"])

#time.sleep(5)

print(robot.position())

"""
move to j0 = 0, ..., j4 = 0
wait for the motion to be done, timeout = 1000 seconds
"""
result = robot.play({"command": "move", "prm":{"path": "joint", "movement":0, "joint":[0, 0, 0, 0, 0]}})
# result = json.loads(result)
# wait = robot._wait_for_command(result, time.time()+1000)


# # move in cartesian space, -10 inches toward X direction
# robot.play({"command": "move", "prm":{"path": "line", "movement":1, "x":-10}})
