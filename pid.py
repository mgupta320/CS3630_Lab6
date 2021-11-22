import cozmo
import threading
import json
import math
import sys
import time
import numpy as np

# Method to move the robot and apply disturbance
# Input
# v_l: (float) left wheel speed
# v_r: (float) right wheel speed
def move(robot: cozmo.robot.Robot, v_l, v_r):
    global start_time, disturbance
    # start_dist is the time when the disturbance is triggered
    start_dist = 0.3
    if start_dist < time.time()-start_time < start_dist + 0.3:
        v_l += disturbance
        v_r += disturbance
    robot.drive_wheel_motors(v_l, v_r)

async def CozmoPID(robot: cozmo.robot.Robot):
    global stopevent, start_time, disturbance
    with open("./config.json") as file:
        config = json.load(file)
    kp = config["kp"]
    ki = config["ki"]
    kd = config["kd"]

    # Reads distrubance from command
    # Default is zero
    disturbance = 0
    if len(sys.argv) > 1:
        disturbance = int(sys.argv[1])
        
    ###############################
    # PLEASE ENTER YOUR CODE BELOW
    ###############################

    # set start_time after Cozmo detects a cube
    start_time = time.time()


class RobotThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.run_program(CozmoPID)
        stopevent.set()


if __name__ == "__main__":
    global stopevent
    stopevent = threading.Event()
    robot_thread = RobotThread()
    robot_thread.start()
    stopevent.set()
