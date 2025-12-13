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

#### LargeMotor

```python
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D

left_motor = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_D)

# Move 360 degrees forward and wait
left_motor.on_for_degrees(speed=50, degrees=360, brake=True, block=True)
right_motor.on_for_degrees(speed=50, degrees=360, brake=True, block=True)
```

#### ColorSensor

```python
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_1

cs = ColorSensor(INPUT_1)
cs.mode = 'COL-REFLECT'

reflection = cs.reflected_light_intensity  # 0=Black, 255=White
```

#### Leds

```python
from ev3dev2.led import Leds

leds = Leds()
leds.set_color('LEFT', 'GREEN')
leds.set_color('RIGHT', 'RED')
```

## Configuration & Parameters

Modify `simulation/robot.py` under **CONFIGURATION CONSTANTS**:

| Constant                  | Description                   |
| ------------------------- | ----------------------------- |
| ROBOT_WIDTH_M             | Robot width (meters)          |
| WHEEL_RADIUS_M            | Wheel radius (meters)         |
| WHEEL_DISTANCE_M          | Distance between wheels       |
| MAX_WHEEL_SPEED_DEG_PER_S | Max motor speed (deg/s)       |
| PIXELS_PER_CM             | Pixels per cm for scaling     |
| SCREEN_WIDTH / HEIGHT     | Pygame window size            |
| LINE_THICKNESS_CM         | Default line thickness in map |



* Obstacles are gray rectangles in the simulation.

## Architecture Overview

Simulator uses three threads:

1. **Physics Thread (Daemon)**: Updates robot position and motor states at `dt=0.01s`.
2. **Rendering Thread (Main)**: Handles Pygame rendering and user input.
3. **Control Logic Thread (Daemon)**: Runs `main()` and safely waits for physics updates on blocking calls.


* Works in both **simulation** and on a real EV3.

## Future Improvements

* Ultrasonic sensor visualization (cone detection)

