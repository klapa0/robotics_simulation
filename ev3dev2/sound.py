import pygame
import math
import numpy as np

class Sound:
    def __init__(self):
        pygame.mixer.init(frequency=44100, size=-16, channels=1)

    def beep(self):
        self.tone(1000, 200)

    def tone(self, frequency, duration_ms):
        sample_rate = 44100
        t = np.linspace(0, duration_ms / 1000, int(sample_rate * duration_ms / 1000), False)
        wave = 0.5 * np.sin(2 * math.pi * frequency * t)
        audio = np.int16(wave * 32767)
        sound = pygame.sndarray.make_sound(audio)
        sound.play()

    def play_file(self, path):
        sound = pygame.mixer.Sound(path)
        sound.play()