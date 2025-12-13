import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# add it to Python's search path
sys.path.append(project_root)
from simulation import robot



INPUT_1 = robot.robot.input[0]
INPUT_2 = robot.robot.input[1]
INPUT_3 = robot.robot.input[2]
INPUT_4 = robot.robot.input[3]
