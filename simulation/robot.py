import threading
import time
import math
import pygame
import numpy as np

# ===========================
# CONFIGURATION CONSTANTS
# ===========================

# Conversion
PIXELS_PER_CM = 7

# Robot physical properties (meters)
ROBOT_WIDTH_M = 0.20
ROBOT_HEIGHT_M = 0.15
WHEEL_RADIUS_M = 0.02
WHEEL_DISTANCE_M = 0.15
MAX_WHEEL_SPEED_DEG_PER_S = 4000
# Robot display (screen) properties
SCREEN_MARGIN_FRONT = 0.01
SCREEN_BORDER_RADIUS = 4
SCREEN_COLOR_BODY = (120, 120, 120)
SCREEN_COLOR_DISPLAY = (170, 200, 220)

# LEDs
LED_WIDTH_RATIO = 1 / 8
LED_HEIGHT_RATIO = 1 / 2
LED_DEFAULT_COLORS = [(0, 0, 0), (255, 0, 0)]

# Wheels
WHEEL_COLOR = (0, 0, 0)

# Map / Pygame window
SCREEN_WIDTH = 1400
SCREEN_HEIGHT = 800
LINE_THICKNESS_CM = 5
LINE_LENGTH_CM = 150

# Gate
DISTANCE_BEETWEEN_OBSTACLES_M = 0.25
OBSTACLES_RADIUS_M = 0.05

# Sensors config
COLOR_SENSOR_DISTANCE_FROM_CENTER_OF_ROBOT = ROBOT_WIDTH_M/2 +0.01
COLOR_SENSOR_RADIUS = 0.01
COLOR_SENSOR_RGB = (255,0,0)

ULTRASONIC_SENSOR_FOV_DEGREES = 20
ULTRASONIC_SENSOR_RGB = (0,255,0)
ULTRASONIC_SENSOR_POSITION_Y_FROM_MIDDLE = COLOR_SENSOR_DISTANCE_FROM_CENTER_OF_ROBOT+0.02

# Ramp / Ball
RAMP_LENGTH_M = 0.10             
RAMP_GEAR_RATIO = 2         
RAMP_MAX_ANGLE_DEG = 120
RAMP_MIN_ANGLE_DEG = -30
RAMP_COLOR = (0, 0, 255)
BALL_RADIUS_M = 0.03            
BALL_COLOR = (255, 100, 0)
GRAVITY_MS2 = 9.81               


RAMP_X_OFFSET_M = -ROBOT_WIDTH_M / 2.5

# ===========================
# HELPER FUNCTIONS
# ===========================

def meters_to_pixels(meters: float) -> int:
    return int(meters * 100 * PIXELS_PER_CM)

def cm_to_pixels(cm: float) -> int:
    return int(cm * PIXELS_PER_CM)

# ===========================
# INPUT / SENSOR CLASS
# ===========================

class Input:
    def __init__(self, robot, position, orientation, size, color, shape):
        self.robot = robot
        self.position = position
        self.theta = orientation
        self.size = size
        self.color = color
        self.shape = shape

    def draw(self, screen):
        world_x, world_y, world_theta = self.get_position_in_pixel()
        if self.shape == 'rect':
            width, height = self.size
            surface = pygame.Surface((width, height), pygame.SRCALPHA)
            surface.fill(self.color)
            rotated = pygame.transform.rotate(surface, -math.degrees(world_theta))
            rect = rotated.get_rect(center=(world_x, world_y))
            screen.blit(rotated, rect.topleft)
        elif self.shape == 'circle':
            radius = self.size * 100 * PIXELS_PER_CM
            pygame.draw.circle(screen, self.color, (int(world_x), int(world_y)), radius)

    def get_position_in_pixel(self):
        dx, dy = self.position
        pixel_x = dx * 100 * PIXELS_PER_CM
        pixel_y = dy * 100 * PIXELS_PER_CM
        theta = self.robot.theta
        world_x = meters_to_pixels(self.robot.x_m) + pixel_x * math.cos(theta) - pixel_y * math.sin(theta)
        world_y = meters_to_pixels(self.robot.y_m) + pixel_x * math.sin(theta) + pixel_y * math.cos(theta)
        return world_x, world_y, self.theta + self.robot.theta
    
    def get_position(self):
        dx, dy = self.position 
        with data_lock:
            rx, ry, r_theta = self.robot.x_m, self.robot.y_m, self.robot.theta
        
        world_x = rx + dx * math.cos(r_theta) - dy * math.sin(r_theta)
        world_y = ry + dx * math.sin(r_theta) + dy * math.cos(r_theta)
        # Zwraca orientację czujnika względem świata
        return world_x, world_y, self.theta + r_theta
    
    def reset(self):
        pass

# ===========================
# OUTPUT / WHEEL CLASS
# ===========================

class Output:
    def __init__(self, robot, wheel_position, wheel_radius):
        self.robot = robot
        self.movement_queue = []  # [speed_percent, target_deg, stop_at_end]
        self.working = False
        self.wheel_position = wheel_position
        self.wheel_radius = wheel_radius
        self.position_deg = 0

    def draw(self, screen):
        world_x, world_y, theta = self.get_position_in_pixel()
        width = self.wheel_radius * 2 * 100 * PIXELS_PER_CM
        height = self.wheel_radius * 100 * PIXELS_PER_CM // 2
        surface = pygame.Surface((width, height), pygame.SRCALPHA)
        surface.fill(WHEEL_COLOR)
        rotated = pygame.transform.rotate(surface, -math.degrees(theta))
        rect = rotated.get_rect(center=(world_x, world_y))
        screen.blit(rotated, rect.topleft)

    def get_position_in_pixel(self):
        dx, dy = self.wheel_position
        pixel_x = dx * 100 * PIXELS_PER_CM
        pixel_y = dy * 100 * PIXELS_PER_CM
        theta = self.robot.theta
        world_x = meters_to_pixels(self.robot.x_m) + pixel_x * math.cos(theta) - pixel_y * math.sin(theta)
        world_y = meters_to_pixels(self.robot.y_m) + pixel_x * math.sin(theta) + pixel_y * math.cos(theta)
        return world_x, world_y, theta

    def add_movement(self, movement):
        self.working = True
        if movement[0] < 0:
            self.movement_queue.append((-movement[0],-movement[1],movement[2]))
        else:
            self.movement_queue.append(movement)

    def move_engine(self, dt):
        if not self.movement_queue:
            self.working = False
            return 0

        old_position = self.position_deg
        speed, target_deg, stop = self.movement_queue[0]

        signed_speed = speed / 100 * MAX_WHEEL_SPEED_DEG_PER_S
        if target_deg < 0:
            signed_speed = -signed_speed 
        
        new_position = self.position_deg + dt * signed_speed

        if stop:
            if target_deg >= 0:
                self.position_deg = min(target_deg, new_position)
            else:
                self.position_deg = max(target_deg, new_position) 
        else:
            self.position_deg = new_position

        difference = self.position_deg - old_position
        
        movement_finished = False
  
        if target_deg >= 0 and self.position_deg >= target_deg:
            movement_finished = True
        elif target_deg < 0 and self.position_deg <= target_deg:
            movement_finished = True
        
        if movement_finished:
            self.position_deg = 0
            with data_lock:
                self.movement_queue.pop(0)
            if not self.movement_queue:
                self.working = False

        return difference

    def reset(self):
        self.movement_queue.clear()
        self.position_deg = 0
        self.working = False

# ===========================
# OUTPUT RAMP CLASS
# ===========================
class OutputBallRamp(Output):
    def __init__(self, robot, wheel_position, wheel_radius, ramp_gear_ratio=RAMP_GEAR_RATIO):
        super().__init__(robot, wheel_position, wheel_radius)
        self.ramp_gear_ratio = ramp_gear_ratio
        
        self.ramp = Ramp(self)
        self.ball = Ball(self.ramp)
        
        self.motor_position_deg = 0

    def add_movement(self, movement):
        super().add_movement(movement)

    def move_engine(self, dt):
        motor_delta_deg = super().move_engine(dt)
        self.motor_position_deg += motor_delta_deg
        
        ramp_delta_deg = motor_delta_deg / self.ramp_gear_ratio
        ramp_delta_rad = math.radians(ramp_delta_deg)
        
        new_angle_rad = self.ramp.current_angle_rad - ramp_delta_rad
        
        min_angle_rad = math.radians(RAMP_MIN_ANGLE_DEG)
        max_angle_rad = math.radians(RAMP_MAX_ANGLE_DEG)
        self.ramp.current_angle_rad = max(min_angle_rad, min(max_angle_rad, new_angle_rad))
        
        self.ball.update_physics(dt)
        
        return motor_delta_deg

    def draw(self, screen):
        self.ramp.draw(screen)
        self.ball.draw(screen)
        pass

    def reset(self):
        super().reset()
        self.ball.reset()
        self.ramp.reset()


class Ball:
    def __init__(self, ramp):
        self.ramp = ramp
        self.position_ratio = 0.0
        self.velocity_m_per_s = 0.0
        self.theta = 0.0
        self.x_m = 0.0
        self.y_m = 0.0
        self.ball_released = False
        
    def update_physics(self, dt):
        if self.ball_released:
            self.x_m+=self.velocity_m_per_s * math.cos(self.theta)*dt
            self.y_m+=self.velocity_m_per_s * math.sin(self.theta)*dt
        else:
            angle_rad = self.ramp.current_angle_rad
            angle_deg = math.degrees(angle_rad)

            if angle_deg >= 0:
                acceleration_m_per_s2 = 0.0
            else:
                slope_angle_rad = abs(angle_rad)
                acceleration_m_per_s2 = GRAVITY_MS2 * math.sin(slope_angle_rad)

            if acceleration_m_per_s2 > 0:
                self.velocity_m_per_s += acceleration_m_per_s2 * dt

            distance_m = self.velocity_m_per_s * dt
            new_position_m = self.position_ratio * RAMP_LENGTH_M + distance_m

            if new_position_m > RAMP_LENGTH_M:
                self.position_ratio = 1.0
                self.ball_released = True
                end_x, end_y, ramp_theta = self.ramp.get_end_position_in_pixel()
                self.theta = ramp_theta
                self.x_m = end_x / PIXELS_PER_CM / 100
                self.y_m = end_y / PIXELS_PER_CM / 100
            else:
                self.position_ratio = new_position_m / RAMP_LENGTH_M

    def draw(self, screen):
        if self.ball_released:
            pygame.draw.circle(screen, BALL_COLOR, (meters_to_pixels(self.x_m), meters_to_pixels(self.y_m)), meters_to_pixels(BALL_RADIUS_M))
        else:
            pivot_x, pivot_y, _ = self.ramp.output.get_position_in_pixel()
            end_x, end_y, ramp_theta = self.ramp.get_end_position_in_pixel()
            angle_for_transform = ramp_theta 

            # Odległość piłki od punktu obrotu
            distance_from_pivot_m = RAMP_LENGTH_M * self.position_ratio 

            # 2. Obliczenie pozycji piłki w świecie (względem pivotu)

            # Piłka porusza się wzdłuż wektora rampy.
            dx_world = distance_from_pivot_m * math.cos(angle_for_transform)
            dy_world = distance_from_pivot_m * math.sin(angle_for_transform)

            world_x = pivot_x + meters_to_pixels(dx_world)
            world_y = pivot_y + meters_to_pixels(dy_world)

            # 3. Rysowanie
            pygame.draw.circle(screen, BALL_COLOR, (int(world_x), int(world_y)), meters_to_pixels(BALL_RADIUS_M))
    
    def reset(self):
        self.position_ratio = 0.0
        self.velocity_m_per_s = 0.0
        self.theta = 0.0
        self.x_m = 0.0
        self.y_m = 0.0
        self.ball_released = False

class Ramp:
    def __init__(self, output_motor):
        self.output = output_motor
        self.current_angle_rad = 1.4
        self.ramp_rotation = math.pi
        
    def get_end_position_in_pixel(self):
        pivot_x, pivot_y, robot_theta = self.output.get_position_in_pixel()
        world_angle =  robot_theta + self.ramp_rotation

        dx = RAMP_LENGTH_M * math.cos(self.ramp_rotation)
        dy = RAMP_LENGTH_M * math.sin(self.ramp_rotation)
        
        dx = meters_to_pixels(dx * math.cos(self.current_angle_rad))
        dy = meters_to_pixels(dy * math.sin(self.current_angle_rad))

        end_x = pivot_x + dx * math.cos(robot_theta) - dy * math.sin(robot_theta)
        end_y = pivot_y + dx * math.sin(robot_theta) + dy * math.cos(robot_theta)
        
        return end_x, end_y, world_angle

    def draw(self, screen):        
        
        pivot_x, pivot_y, robot_theta = self.output.get_position_in_pixel()
        end_x, end_y, world_angle       =  self.get_end_position_in_pixel()
        total_angle_rad =  world_angle
        total_angle_deg = -math.degrees(total_angle_rad) 
        
        length_px = int(max(5,abs(meters_to_pixels(RAMP_LENGTH_M) * math.cos(self.current_angle_rad))))
        thickness_px = meters_to_pixels(0.015) 

        base_surface = pygame.Surface((length_px, thickness_px), pygame.SRCALPHA)
        pygame.draw.rect(base_surface, (0,0,0), (0, 0, length_px, thickness_px))
        pygame.draw.rect(base_surface, RAMP_COLOR, (0, meters_to_pixels(0.0029), length_px, meters_to_pixels(0.015-0.005)))
        
        rotated = pygame.transform.rotate(base_surface, total_angle_deg)
        rect = rotated.get_rect(center=(pivot_x-(pivot_x-end_x)/2, (pivot_y-(pivot_y-end_y)/2)))
        screen.blit(rotated, rect.topleft)
    
    def reset(self):
        self.current_angle_rad = 1.4



    
# ===========================
# ROBOT SIMULATION CLASS
# ===========================

class RobotSim:
    def __init__(self):
        self.x_m = 0.1
        self.y_m = 0.1
        self.theta = 0.7
        self.set_robot_orientation_degrees(45)
        self.width = ROBOT_WIDTH_M
        self.height = ROBOT_HEIGHT_M
        self.wheel_radius = WHEEL_RADIUS_M
        self.wheel_distance = WHEEL_DISTANCE_M

        self.input = [None] * 4
        self.output = [None] * 4
        self.movementWheel0 = -1
        self.movementWheel1 = -1
        self.mediumEngine = -1

        self.led_color = LED_DEFAULT_COLORS.copy()
    
    def set_robot_orientation_degrees(self,degrees):
        self.theta = (degrees / 180.0) * math.pi

    def draw(self, screen):
        #robot.output[2] = OutputBallRamp(robot, (RAMP_X_OFFSET_M, 0), 0) Create robot body surface
        surface = pygame.Surface((meters_to_pixels(self.width), meters_to_pixels(self.height)), pygame.SRCALPHA)
        surface.fill(SCREEN_COLOR_BODY)

        self._draw_screen(surface)
        self._draw_leds(surface)

        # Rotate robot according to theta
        rotated = pygame.transform.rotate(surface, -math.degrees(self.theta))
        rect = rotated.get_rect(center=(meters_to_pixels(self.x_m), meters_to_pixels(self.y_m)))
        screen.blit(rotated, rect.topleft)

        # Draw inputs and outputs
        for sensor in self.input:
            if sensor: sensor.draw(screen)
        for wheel in self.output:
            if wheel: wheel.draw(screen)

    def _draw_screen(self, surface):
        screen_w = meters_to_pixels(self.width/2 - SCREEN_MARGIN_FRONT)
        screen_h = meters_to_pixels(self.height - 0.02)
        screen_x = meters_to_pixels(self.width) - screen_w
        screen_y = cm_to_pixels(1)

        pygame.draw.rect(surface, (20, 20, 20), (screen_x, screen_y, screen_w, screen_h), border_radius=SCREEN_BORDER_RADIUS)
        pygame.draw.rect(surface, SCREEN_COLOR_DISPLAY, (screen_x+2, screen_y+2, screen_w-4, screen_h-4), border_radius=SCREEN_BORDER_RADIUS-1)

    def _draw_leds(self, surface):
        led_w = meters_to_pixels(self.width * LED_WIDTH_RATIO)
        led_h = led_w // 2
        led_y = meters_to_pixels(self.height / 2)

        for i, color in enumerate(self.led_color):
            offset = (i - 0.5) * led_h * 2
            pygame.draw.rect(surface, color, (meters_to_pixels(self.width/2) - led_w, led_y + offset, led_w, led_h), border_radius=4)

    def update_physics(self, dt):
        if self.mediumEngine >=0:
            self.output[self.mediumEngine].move_engine(dt)
        """Update robot position using differential drive model"""
        delta_left = self.output[self.movementWheel0].move_engine(dt)
        delta_right = self.output[self.movementWheel1].move_engine(dt)

        v_left = math.radians(delta_left) * self.wheel_radius 
        v_right = math.radians(delta_right) * self.wheel_radius 

        v = (v_right + v_left) / 2
        omega = (v_right - v_left) / self.wheel_distance

        with data_lock:
            self.theta += omega
            self.x_m += v * math.cos(self.theta)
            self.y_m += v * math.sin(self.theta)
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # normalize
    
    def reset(self):
        for input in self.input:
            if input:
                input.reset()
        for output in self.output:
            if output:
                output.reset()
    
    def set_position_m_from_line(self,y):
        line_position = (SCREEN_HEIGHT / 2 / PIXELS_PER_CM / 100)
        self.y_m = line_position - y


# ===========================
# INITIALIZE ROBOT
# ===========================

data_lock = threading.Lock()
robot = RobotSim()

# Initialize wheels
robot.output[0] = Output(robot, (0, robot.wheel_distance/2), robot.wheel_radius)
robot.movementWheel0 = 0
robot.output[3] = Output(robot, (0, -robot.wheel_distance/2), robot.wheel_radius)
robot.movementWheel1 = 3

robot.output[1] = OutputBallRamp(robot, (RAMP_X_OFFSET_M, 0), 0)
robot.mediumEngine = 1
# Initialize color sensor
robot.input[0] = Input(robot, ((COLOR_SENSOR_DISTANCE_FROM_CENTER_OF_ROBOT),0), 0, COLOR_SENSOR_RADIUS, COLOR_SENSOR_RGB, 'circle')
robot.input[3] = Input(robot, ((ULTRASONIC_SENSOR_POSITION_Y_FROM_MIDDLE),0), 0, 0.01, ULTRASONIC_SENSOR_RGB, 'circle')

# Start physics simulation thread
def _simulation_loop():
    dt = 0.01
    while True:
        robot.update_physics(dt)
        time.sleep(dt)

_sim_thread = threading.Thread(target=_simulation_loop, daemon=True)
_sim_thread.start()

# ===========================
# MAP / ENVIRONMENT SETUP
# ===========================

map_array = np.full((SCREEN_WIDTH, SCREEN_HEIGHT), 255, dtype=np.uint8)
center_x = SCREEN_HEIGHT // 2
line_thickness_px = cm_to_pixels(LINE_THICKNESS_CM)
line_width_px = cm_to_pixels(LINE_LENGTH_CM)
map_array[0:line_width_px, center_x - line_thickness_px//2:center_x + line_thickness_px//2] = 0
rgb_map = np.stack([map_array]*3, axis=-1)
map_surface = pygame.surfarray.make_surface(rgb_map)
pygame.surfarray.blit_array(map_surface, rgb_map)

class Obstacle:
    """Represents a rectangular obstacle in the world"""
    def __init__(self, x, y, radius):
        # x, y = center of the obstacle
        self.x = x
        self.y = y
        self.radius = radius

    def get_corners(self):
        """Return list of 4 corners of the rectangle"""
        return [
            (self.x - self.radius, self.y - self.radius),
            (self.x + self.radius, self.y - self.radius),
            (self.x + self.radius, self.y + self.radius),
            (self.x - self.radius, self.y + self.radius)
        ]

    def draw(self, screen):
        pygame.draw.circle(screen, (130,200,130), (int(meters_to_pixels(self.x)), int(meters_to_pixels(self.y))), meters_to_pixels(self.radius))

obstacles = [
    Obstacle(LINE_LENGTH_CM/100, SCREEN_HEIGHT/2 /PIXELS_PER_CM/100 - DISTANCE_BEETWEEN_OBSTACLES_M/2-OBSTACLES_RADIUS_M, OBSTACLES_RADIUS_M),
    Obstacle(LINE_LENGTH_CM/100, SCREEN_HEIGHT/2 /PIXELS_PER_CM/100 + DISTANCE_BEETWEEN_OBSTACLES_M/2+OBSTACLES_RADIUS_M, OBSTACLES_RADIUS_M),
]
# ===========================
# START PYGAME
# ===========================

def start():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Robot Simulation")
    clock = pygame.time.Clock()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.blit(map_surface, (0, 0))
        for obs in obstacles:
            obs.draw(screen)
        with data_lock:
            robot.draw(screen)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
