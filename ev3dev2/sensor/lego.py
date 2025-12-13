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
        cx, cy, _ = self.input.get_position_in_pixel()
        r_px = max(1, int(self.input.size * 100 * robot.PIXELS_PER_CM))
        min_x = max(0, int(cx - r_px))
        max_x = min(robot.SCREEN_WIDTH, int(cx + r_px))
        min_y = max(0, int(cy - r_px))
        max_y = min(robot.SCREEN_HEIGHT, int(cy + r_px))
        total = 0.0
        count = 0
        
        for x in range(min_x, max_x):
            for y in range(min_y, max_y):
                if (x - cx)**2 + (y - cy)**2 <= r_px**2:
                    value = robot.map_array[x, y]
                    total += value
                    count += 1

        mean_val = total / count if count > 0 else 255
        self._reflected_light_intensity = int(round((mean_val / 255) * 100))


    @property
    def reflected_light_intensity(self):
        self.update()  
        return self._reflected_light_intensity

import math

class UltrasonicSensor:
    def __init__(self, input):
        self.input = input
        self.mode = 'US-DIST-CM'
        self.fov = math.radians(robot.ULTRASONIC_SENSOR_FOV_DEGREES)
        
    def _calculate_distance(self):
        sensor_x_m, sensor_y_m, sensor_world_theta = self.input.get_position()
        min_dist_m = float('inf')
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
