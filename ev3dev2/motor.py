import sys
import os
import time
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# add it to Python's search path
sys.path.append(project_root)

# now you can import robot.py
from simulation import robot


class LargeMotor:
    def __init__(self, output):
        self.output = output
    
    def on_for_degrees(self,speed = 10, degrees = 0, brake = True, block = True):
        with robot.data_lock:
            self.output.addEngineMove((speed,degrees,brake))
        if block:
            while self.output.isWorking():
                time.sleep(0.01)

OUTPUT_A = robot.robot.output[0]
OUTPUT_B = robot.robot.output[1]
OUTPUT_C = robot.robot.output[2]
OUTPUT_D = robot.robot.output[3]