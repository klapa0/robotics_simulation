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
WHEEL_RADIUS_M = 0.05
WHEEL_DISTANCE_M = 0.15
MAX_WHEEL_SPEED_DEG_PER_S = 4000
COLOR_SENSOR_DISTANCE_FROM_CENTER_OF_ROBOT = ROBOT_WIDTH_M/2 +0.01
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
LINE_LENGTH_CM = 100

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
        world_x, world_y, worlSCREEN_HEIGHTd_theta = self.get_position_in_pixel()
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
        world_x = self.robot.x + pixel_x * math.cos(theta) - pixel_y * math.sin(theta)
        world_y = self.robot.y + pixel_x * math.sin(theta) + pixel_y * math.cos(theta)
        return world_x, world_y, self.theta + self.robot.theta

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
        world_x = self.robot.x + pixel_x * math.cos(theta) - pixel_y * math.sin(theta)
        world_y = self.robot.y + pixel_x * math.sin(theta) + pixel_y * math.cos(theta)
        return world_x, world_y, theta

    def add_movement(self, movement):
        self.working = True
        self.movement_queue.append(movement)

    def move_engine(self, dt):
        if not self.movement_queue:
            self.working = False
            return 0

        old_position = self.position_deg
        speed, target_deg, stop = self.movement_queue[0]

        if stop:
            if target_deg >= 0:
                self.position_deg = min(target_deg, self.position_deg + dt * speed/100 * MAX_WHEEL_SPEED_DEG_PER_S)
            else:
                self.position_deg = max(target_deg, self.position_deg + dt * speed/100 * MAX_WHEEL_SPEED_DEG_PER_S)
        else:
            self.position_deg += dt * speed/100 * MAX_WHEEL_SPEED_DEG_PER_S

        difference = self.position_deg - old_position

        if self.position_deg >= target_deg:
            self.position_deg = 0
            with data_lock:
                self.movement_queue.pop(0)
            if not self.movement_queue:
                self.working = False

        return difference

# ===========================
# ROBOT SIMULATION CLASS
# ===========================

class RobotSim:
    def __init__(self):
        self.x = 100.0
        self.y = 100.0
        self.theta = 0.7
        self.width = ROBOT_WIDTH_M
        self.height = ROBOT_HEIGHT_M
        self.wheel_radius = WHEEL_RADIUS_M
        self.wheel_distance = WHEEL_DISTANCE_M

        self.input = [None] * 4
        self.output = [None] * 4
        self.movementWheel0 = -1
        self.movementWheel1 = -1

        self.led_color = LED_DEFAULT_COLORS.copy()

    def draw(self, screen):
        # Create robot body surface
        surface = pygame.Surface((meters_to_pixels(self.width), meters_to_pixels(self.height)), pygame.SRCALPHA)
        surface.fill(SCREEN_COLOR_BODY)

        self._draw_screen(surface)
        self._draw_leds(surface)

        # Rotate robot according to theta
        rotated = pygame.transform.rotate(surface, -math.degrees(self.theta))
        rect = rotated.get_rect(center=(self.x, self.y))
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
        """Update robot position using differential drive model"""
        delta_left = self.output[self.movementWheel0].move_engine(dt)
        delta_right = self.output[self.movementWheel1].move_engine(dt)

        v_left = math.radians(delta_left) * self.wheel_radius / dt
        v_right = math.radians(delta_right) * self.wheel_radius / dt

        v = (v_right + v_left) / 2
        omega = (v_right - v_left) / self.wheel_distance

        with data_lock:
            self.theta += omega * dt
            self.x += v * math.cos(self.theta)
            self.y += v * math.sin(self.theta)
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # normalize

# ===========================
# INITIALIZE ROBOT
# ===========================

data_lock = threading.Lock()
robot = RobotSim()

# Initialize wheels
robot.output[0] = Output(robot, (0, robot.wheel_distance/2), robot.wheel_radius)
robot.movementWheel1 = 0
robot.output[3] = Output(robot, (0, -robot.wheel_distance/2), robot.wheel_radius)
robot.movementWheel0 = 3

# Initialize color sensor
robot.input[0] = Input(robot, ((COLOR_SENSOR_DISTANCE_FROM_CENTER_OF_ROBOT),0), 0, 0.01, (255,0,0), 'circle')

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
    def __init__(self, x, y, width, height):
        # x, y = center of the obstacle
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def get_corners(self):
        """Return list of 4 corners of the rectangle"""
        hw, hh = self.width/2, self.height/2
        return [
            (self.x - hw, self.y - hh),
            (self.x + hw, self.y - hh),
            (self.x + hw, self.y + hh),
            (self.x - hw, self.y + hh)
        ]

    def draw(self, screen):
        rect = pygame.Rect(
            meters_to_pixels(self.x - self.width/2),
            meters_to_pixels(self.y - self.height/2),
            meters_to_pixels(self.width),
            meters_to_pixels(self.height)
        )
        pygame.draw.rect(screen, (100,100,100), rect)
obstacles = [
    Obstacle(LINE_LENGTH_CM/100, SCREEN_HEIGHT/2 /PIXELS_PER_CM/100 - 0.18, 0.1, 0.1),
    Obstacle(LINE_LENGTH_CM/100, SCREEN_HEIGHT/2 /PIXELS_PER_CM/100 + 0.18, 0.1, 0.1),
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
