import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)
from simulation import robot



EV3_COLORS = {
    'BLACK':  (0, 0, 0),
    'RED':    (255, 0, 0),
    'GREEN':  (0, 255, 0),
    'AMBER':  (255, 191, 0),
    'ORANGE': (255, 140, 0),
    'YELLOW': (255, 255, 0),
}


class Leds:
    def __init__(self):
        pass

    def set_color(self, whichLed, color, pct=None):
        if whichLed == 'LEFT':
            idx = 0
        elif whichLed == 'RIGHT':
            idx = 1
        else:
            raise ValueError("LED must be 'LEFT' or 'RIGHT'")

        if isinstance(color, str):
            rgb = EV3_COLORS[color]

        elif isinstance(color, tuple):
            if len(color) == 2:
                r, g = color
                b = 0
            else:
                r, g, b = color

            rgb = (
                int(255 * r),
                int(255 * g),
                int(255 * b),
            )
        else:
            raise ValueError("Color must be str or tuple")

        if pct is not None:
            rgb = tuple(int(c * pct) for c in rgb)

        robot.robot.led_color[idx] = rgb
