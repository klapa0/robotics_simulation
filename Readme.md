# EV3-Python 2D Robot Simulator

A lightweight, 2D physics-based robot simulator written in Python. This project mocks the `ev3dev2` library, allowing you to run and test LEGO Mindstorms EV3 programs directly on your computer without physical hardware. It uses Pygame for visualization and NumPy for environment mapping.

## Features

* Mock `ev3dev2` API: Use classes like `LargeMotor`, `ColorSensor`, and `Leds` like on a real EV3.
* Differential Drive Physics: Simulates realistic robot movement based on wheel speed and geometry.
* Visual Environment: Real-time rendering of robot, sensors, and obstacles.
* Threaded Architecture: Separates physics, rendering, and control logic for stability.


## How to Run

1. Open `main.py` and ensure `SIMULATION = True`.
2. Run the script from project root:

```bash
python main.py
```

* A Pygame window opens and your robot code begins executing.

## Writing Robot Code

Write control logic inside `main()` in `main.py`. The simulator handles threading, so blocking calls (like `motor.on_for_degrees(..., block=True)`) work without freezing the window.

### Supported Classes

#### LargeMotor / MediumMotor (Drivetrain and Ramp)

```python
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_D, OUTPUT_B

# Motors for wheels
left_motor = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_D)

# Motor for the ramp
ramp_motor = MediumMotor(OUTPUT_B)

# Example: Move 360 degrees forward
left_motor.on_for_degrees(speed=50, degrees=360, brake=True, block=True)
right_motor.on_for_degrees(speed=50, degrees=360, brake=True, block=True)

# Example: Tilt ramp 90 degrees (degrees are relative to motor's initial position)
# NOTE: The motor angle is divided by RAMP_GEAR_RATIO to get the ramp's angle.
ramp_motor.on_for_degrees(speed=10, degrees=90, brake=True, block=True)
```

#### ColorSensor

```python
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_1

cs = ColorSensor(INPUT_1)
cs.mode = 'COL-REFLECT'

reflection = cs.reflected_light_intensity  
```
### UltrasonicSensor

```python
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor import INPUT_4

us = UltrasonicSensor(INPUT_4)
# Returns distance to the nearest obstacle within the sensor's FOV in centimeters
distance_cm = us.distance_centimeters
```

#### Leds

```python
from ev3dev2.led import Leds

leds = Leds()
leds.set_color('LEFT', 'GREEN')
leds.set_color('RIGHT', 'RED')
```
### Sound

```python
from ev3dev2.sound import Sound

sound = Sound()
sound.beep()                # Short beep
sound.tone(1000, 200)       # Tone at 1000 Hz for 200 ms
```

## Configuration & Parameters

Modify `simulation/robot.py` under **CONFIGURATION CONSTANTS**:

| Constant                  | Description                   |
| ------------------------- | ----------------------------- |
| PIXELS_PER_CM                             |Scaling factor for visualization (pixels per cm)|
| ROBOT_WIDTH_M                             |Width of the robot body (m)|
| ROBOT_HEIGHT_M                            |Length or depth of the robot body (m)|
| WHEEL_RADIUS_M                            |Radius of the drive wheels (m)|
| WHEEL_DISTANCE_M                          |Distance between the centers of the drive wheels (m)|
| MAX_WHEEL_SPEED_DEG_PER_S                 |Maximum speed of the motors (degr per second)|
| SCREEN_MARGIN_FRONT                       |Margin for the robot's display screen at the front (m)|
| SCREEN_BORDER_RADIUS                      |Corner radius of the robot's display screen|
| SCREEN_COLOR_BODY                         |RGB color of the main robot chassis|
| SCREEN_COLOR_DISPLAY                      |RGB background color of the robot's display screen|
| LED_WIDTH_RATIO                           |Width of the LED relative to robot width|
| LED_HEIGHT_RATIO                          |Height of the LED relative to robot height|
| LED_DEFAULT_COLORS                        |Default RGB colors for the left and right LEDs|
| WHEEL_COLOR                               |RGB color used to draw the wheels|
| SCREEN_WIDTH                              |Width of the Pygame window (pixels)|
| SCREEN_HEIGHT                             |Height of the Pygame window (pixels)|
| LINE_THICKNESS_CM                         |Thickness of the line drawn on the map (cm)|
| LINE_LENGTH_CM                            |Length of the line drawn on the map (cm)|
| DISTANCE_BEETWEEN_OBSTACLES_M             |Distance between the obstacles forming the gate (m)|
| OBSTACLES_RADIUS_M                        |Radius of the individual obstacles (m)|
| COLOR_SENSOR_DISTANCE_FROM_CENTER_OF_ROBOT|X-axis position of the color sensor relative to the robot center (m)|
| COLOR_SENSOR_RADIUS                       |Radius of the color sensor for scanning area and visualization (m)|
| COLOR_SENSOR_RGB                          |RGB color of the color sensor visualization|
| ULTRASONIC_SENSOR_FOV_DEGREES             |Field of View (FOV) angle of the ultrasonic sensor (degr)|
| ULTRASONIC_SENSOR_RGB                     |RGB color of the ultrasonic sensor visualization|
| ULTRASONIC_SENSOR_POSITION_Y_FROM_MIDDLE  |Y-axis position of the ultrasonic sensor relative to the robot center (m)|
| RAMP_LENGTH_M                             |Length of the ramp arm (m)|
| RAMP_GEAR_RATIO                           |Gear ratio applied to the ramp motor (Motor Degrees / Ramp Angle Degrees)|
| RAMP_MAX_ANGLE_DEG                        |Maximum lowering angle of the ramp (degr)|
| RAMP_MIN_ANGLE_DEG                        |Minimum raising angle of the ramp (degr)|
| RAMP_COLOR                                |RGB color of the ramp visualization|
| BALL_RADIUS_M                             |Radius of the ball (m)|
| BALL_COLOR                                |RGB color of the ball visualization|
| GRAVITY_MS2                               |Gravitational acceleration for ball physics (m/s²)|
| RAMP_X_OFFSET_M                           |X-axis position of the ramp pivot point relative to the robot center (m)|


* Obstacles are gray rectangles in the simulation.
### Testing & Automation
Use the test() function in main_thread to run predefined positions and rotations:
```python
from simulation.robot import test

tests = [(0.1, 0.3, 45), (0.1, -0.3, -45)]
test(my_robot_function, 0.1)
```
This resets the robot before each run and waits sleep_time seconds between tests.

## Architecture Overview

Simulator uses three threads:

1. **Physics Thread (Daemon)**: Updates robot position and motor states at `dt=0.01s`.
2. **Rendering Thread (Main)**: Handles Pygame rendering and user input.
3. **Control Logic Thread (Daemon)**: Runs `main()` and safely waits for physics updates on blocking calls.


* Works in both **simulation** and on a real EV3.

## Future Improvements

* Ultrasonic sensor visualization (cone detection)

