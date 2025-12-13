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

import math

class UltrasonicSensor:
    def __init__(self, input):
        self.input = input
        self.mode = 'US-DIST-CM'
        self.fov = math.radians(20)
        
    def _calculate_distance(self):
        sensor_x_m, sensor_y_m, sensor_world_theta = self.input.get_position()
        min_dist_m = float('inf')
        print(self.input.get_position())
        for obs in robot.obstacles:
            points = obs.get_corners() 
            
            for px_m, py_m in points:
                dx_m = px_m - sensor_x_m
                dy_m = py_m - sensor_y_m
                angle_to_point = math.atan2(dy_m, dx_m)
                rel_angle = angle_to_point - sensor_world_theta
                rel_angle = math.atan2(math.sin(rel_angle), math.cos(rel_angle))
                if abs(rel_angle) <= self.fov / 2:
                    dist_m = math.hypot(dx_m, dy_m)
                    if dist_m < min_dist_m:
                        min_dist_m = dist_m

        
        if min_dist_m == float('inf'):
             return 2.5
        

        return min_dist_m * 100.0

    @property
    def distance_centimeters_continuous(self):
        return self._calculate_distance()

    @property
    def distance_centimeters(self):
        return self._calculate_distance()
