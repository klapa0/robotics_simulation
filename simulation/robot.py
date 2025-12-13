import threading
import time
import math
import pygame

HOW_MANY_PIXELS_ONE_CENTIMETER = 7
data_lock = threading.Lock()


class Input:
    def __init__(self, robot ,position, orientation, figure, color, shape):
        self.robot = robot
        self.position = position
        self.theta = orientation
        self.figure = figure
        self.color = color
        self.shape = shape

    def draw(self, screen):
        world_x, world_y, world_theta = self.getPositionInPixel()
        if self.shape == 'rect':
            width, height = self.figure
            surface = pygame.Surface((width, height), pygame.SRCALPHA)
            surface.fill(self.color)
            rotated = pygame.transform.rotate(surface, -math.degrees(world_theta))
            rect = rotated.get_rect(center=(world_x, world_y))
            screen.blit(rotated, rect.topleft)
        elif self.shape == 'circle':
            pygame.draw.circle(screen, self.color, (int(world_x), int(world_y)), self.figure*100*HOW_MANY_PIXELS_ONE_CENTIMETER)

    def getPositionInPixel(self):
        dx, dy = self.position
        pixel_x = dx * HOW_MANY_PIXELS_ONE_CENTIMETER * 100
        pixel_y = dy * HOW_MANY_PIXELS_ONE_CENTIMETER * 100
        theta = self.robot.theta
        world_x = self.robot.x + pixel_x * math.cos(theta) - pixel_y * math.sin(theta)
        world_y = self.robot.y + pixel_x * math.sin(theta) + pixel_y * math.cos(theta)
        return world_x, world_y, self.theta + self.robot.theta
    
    def getFigure(self):
        if self.shape == 'circle':
            return self.figure*100*HOW_MANY_PIXELS_ONE_CENTIMETER
    
MAX_SPEED_DEGREES_IN_S = 4000

class Output:
    def __init__(self, robot ,wheel_position, wheel_radius):
        self.robot = robot
        #speed and degrees are here
        self.movement_queue = []
        self.works = False
        #position in pixels relative to main robot
        self.wheel_position = wheel_position
        #radius of the wheel in centimeters
        self.wheel_radius = wheel_radius
        self.position = 0

    def draw(self, screen):
        world_x, world_y, theta = self.getPositionInPixel()
        
        # parametry prostokąta reprezentującego koło
        width = self.wheel_radius * 2  * 100 * HOW_MANY_PIXELS_ONE_CENTIMETER      # długość wzdłuż osi koła
        height = (self.wheel_radius * 100 * HOW_MANY_PIXELS_ONE_CENTIMETER) // 2       # krótsza część prostokąta
        
        # tworzymy powierzchnię prostokąta z przezroczystością
        surface = pygame.Surface((width, height), pygame.SRCALPHA)
        surface.fill((0, 0, 0))  # kolor koła szary

        # obracamy prostokąt zgodnie z orientacją robota
        rotated = pygame.transform.rotate(surface, -math.degrees(theta))

        # pozycjonujemy prostokąt względem środka robota
        rect = rotated.get_rect(center=(world_x, world_y))
        screen.blit(rotated, rect.topleft)

    def getPositionInPixel(self):
        """Zwraca pozycję koła w układzie świata"""
        dx, dy = self.wheel_position
        pixel_x = dx * HOW_MANY_PIXELS_ONE_CENTIMETER * 100
        pixel_y = dy * HOW_MANY_PIXELS_ONE_CENTIMETER * 100
        theta = self.robot.theta
        world_x = self.robot.x + pixel_x * math.cos(theta) - pixel_y * math.sin(theta)
        world_y = self.robot.y + pixel_x * math.sin(theta) + pixel_y * math.cos(theta)
        return world_x, world_y, theta

    def isWorking(self):
        return self.works
    
    def addEngineMove(self, movement):
        #appends speed and degrees and if stop at finish (speed from 0 to 100 as power, degrees, stop)
        self.works = True
        self.movement_queue.append(movement)

    def moveEngine(self, dt):
        if len(self.movement_queue) == 0:
            self.works = False
            return 0
        
        old_position = self.position

        if self.movement_queue[0][2]:
            if self.movement_queue[0][1] >= 0:
                self.position = min(self.movement_queue[0][1], self.position + dt*self.movement_queue[0][0]/100 * MAX_SPEED_DEGREES_IN_S )
            elif self.movement_queue[0][1] < 0:
                self.position = max(self.movement_queue[0][1], self.position + dt*self.movement_queue[0][0]/100 * MAX_SPEED_DEGREES_IN_S )
        else:
            self.position = self.position + dt*self.movement_queue[0][0]/100 * MAX_SPEED_DEGREES_IN_S
        
        difference = self.position - old_position

        if self.position >= self.movement_queue[0][1]:
            self.position = 0
            with data_lock:
                self.movement_queue.pop(0)
            if len(self.movement_queue) == 0:
                self.works = False
        
        return difference

class RobotSim:
    def __init__(self):
        self.x = 100.0
        self.y = 100.0
        self.theta = 0.7
        self.input = [None,None,None,None]

        self.output = [None,None,None,None]
        self.width = 0.20  # szerokość robota w pikselach
        self.height = 0.15
        self.robot_wheels_size_m = 0.05
        self.robot_wheels_distance_m = 0.15
        self.movementWheel1 = -1
        self.movementWheel0 = -1
        self.led_active = True
        self.led_color = [(0,0,0),(255,0,0)]
        # motors, sensors etc.

    def draw(self, screen):
        # tworzymy powierzchnię prostokąta robota
        surface = pygame.Surface((self.width*HOW_MANY_PIXELS_ONE_CENTIMETER*100, self.height*HOW_MANY_PIXELS_ONE_CENTIMETER*100), pygame.SRCALPHA)
        surface.fill((120, 120, 120))  # szary kolor

        screen_w = int((self.width/2-0.01) * HOW_MANY_PIXELS_ONE_CENTIMETER*100)   # 6 cm
        screen_h = int((self.height-0.02) * HOW_MANY_PIXELS_ONE_CENTIMETER*100)   # 3 cm

        screen_x = self.width * HOW_MANY_PIXELS_ONE_CENTIMETER*100 - screen_w
        screen_y = int(0.01 * HOW_MANY_PIXELS_ONE_CENTIMETER*100)   # lekko od przodu

        pygame.draw.rect(
            surface,
            (20, 20, 20),                 # obudowa
            (screen_x, screen_y, screen_w, screen_h),
            border_radius=4
        )

        pygame.draw.rect(
            surface,
            (170, 200, 220),                # „wyświetlacz”
            (screen_x+2, screen_y+2, screen_w-4, screen_h-4),
            border_radius=3
        )

        led1_w = int((self.width/8) * HOW_MANY_PIXELS_ONE_CENTIMETER*100)
        led1_h = led1_w//2
        led1_y = self.height * HOW_MANY_PIXELS_ONE_CENTIMETER * 100/2
        pygame.draw.rect(
            surface,
            self.led_color[1],                 # obudowa
            (screen_x-led1_w, led1_y +led1_h/2, led1_w, led1_h),
            border_radius=4
        )
        pygame.draw.rect(
            surface,
            self.led_color[0],                 # obudowa
            (screen_x-led1_w, led1_y -led1_h/2, led1_w, led1_h),
            border_radius=4
        )
        # obracamy prostokąt zgodnie z orientacją robota
        rotated = pygame.transform.rotate(surface, -math.degrees(self.theta))

        # pozycjonujemy prostokąt względem środka robota
        rect = rotated.get_rect(center=(self.x, self.y))
        screen.blit(rotated, rect.topleft)

        # rysujemy sensory
        for s in self.input:
            if s:
                s.draw(screen)
        # rysujemy koła
        for w in self.output:
            if w:
                w.draw(screen)

    def getPositionInPixel(self):
        return(self.x * 100 * HOW_MANY_PIXELS_ONE_CENTIMETER,self.y * 100 * HOW_MANY_PIXELS_ONE_CENTIMETER, self.theta)
    
    def update_physics(self, dt):
        """Oblicza nową pozycję na podstawie prędkości kół (Model Differential Drive)"""
        # Pobranie prędkości liniowych kół (V = omega * r)
        delta_left_deg = self.output[self.movementWheel0].moveEngine(dt)
        delta_right_deg = self.output[self.movementWheel1].moveEngine(dt)

        # prędkość liniowa kół w m/s
        v_left = math.radians(delta_left_deg) * self.robot_wheels_size_m / dt
        v_right = math.radians(delta_right_deg) * self.robot_wheels_size_m / dt

        # prędkość robota i prędkość kątowa
        v = (v_right + v_left) / 2
        omega = (v_right - v_left) / self.robot_wheels_distance_m 

        with data_lock:
            self.theta += omega * dt
            self.x += v * math.cos(self.theta)
            self.y += v * math.sin(self.theta)

            # Normalizacja kąta (-pi do pi)
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))



robot = RobotSim()

robot.output[0] = Output(robot, (0,robot.robot_wheels_distance_m/2), robot.robot_wheels_size_m)
robot.movementWheel1 = 0
robot.output[3] = Output(robot, (0,-robot.robot_wheels_distance_m/2), robot.robot_wheels_size_m)
robot.movementWheel0 = 3

robot.input[0] = Input(robot, ((robot.width/2 +0.01),0), 0, 0.01,(255,0,0),'circle')



# start simulation thread
def _simulation_loop():
    dt = 0.01
    while True:
        # update motors and robot pose
        robot.update_physics(dt)
        time.sleep(dt)

_sim_thread = threading.Thread(target=_simulation_loop, daemon=True)
_sim_thread.start()



import numpy as np

SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 800


# Tworzymy mapę całego ekranu
# 0 = czarny (linia), 255 = biały (tło)
map_array = np.full((SCREEN_WIDTH, SCREEN_HEIGHT), 255, dtype=np.uint8)

# narysuj pionową linię przez środek
center_x = SCREEN_HEIGHT // 2
line_thickness = 5 * HOW_MANY_PIXELS_ONE_CENTIMETER
line_width = 50 * HOW_MANY_PIXELS_ONE_CENTIMETER
map_array[0:line_width, center_x - line_thickness//2 : center_x + line_thickness//2] = 0

# Konwersja mapy na powierzchnię Pygame
# musimy zamienić 2D array na 3 kanały RGB
rgb_map = np.stack([map_array]*3, axis=-1)  # shape (H,W,3)
map_surface = pygame.surfarray.make_surface(rgb_map)
pygame.surfarray.blit_array(map_surface, rgb_map)




def start():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Robot Simulation")
    clock = pygame.time.Clock()

    running = True
    while running:
        # obsługa zdarzeń
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # czyszczenie ekranu
        screen.blit(map_surface, (0, 0))

        # rysowanie robota i jego elementów
        with data_lock:  # blokada, żeby uniknąć konfliktów z wątkiem symulacji
            robot.draw(screen)

        # aktualizacja ekranu
        pygame.display.flip()

        # limit FPS
        clock.tick(60)

    pygame.quit()


