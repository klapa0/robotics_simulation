import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)
from simulation import robot

class ColorSensor:
    def __init__(self, input):
        self.input = input
        self.mode = ''
        self._reflected_light_intensity = 0

    def update(self):
        x,y,_ = self.input.get_position_in_pixel()
        radius = self.input.size
        ix = int(x)
        iy = int(y)
        self._reflected_light_intensity = robot.map_array[ix, iy]


    @property
    def reflected_light_intensity(self):
        self.update()  
        return self._reflected_light_intensity