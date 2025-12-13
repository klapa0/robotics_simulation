#!/usr/bin/env python3
SIMULATION = True
PI = 3.141592653589793
WHEEL_RADIUS_M = 0.02
WHEEL_DISTANCE_M = 0.15

from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D
from ev3dev2.led import Leds
from ev3dev2.sensor.lego import UltrasonicSensor
from time import sleep
large_motor_B = LargeMotor(OUTPUT_A)
large_motor_C = LargeMotor(OUTPUT_D)
grados180 = 195
forward = 1

sensor = ColorSensor(INPUT_1)
sensor.mode = 'COL-REFLECT'   # Lectura de luz reflejada (0–100)

distance_sensor = UltrasonicSensor(INPUT_4)
distance_sensor.mode = 'US-DIST-CM'

def move_cm(motor1,motor2,speed,cm,brake,block):
    rotation_deg = (cm * 360) / (100 * 2 * PI * WHEEL_RADIUS_M)
    print(rotation_deg)
    moveForward(motor1, motor2, speed, rotation_deg, brake, block)

def rotate_robot_degrees(motor1,motor2,speed,angle_deg,brake,block):
    degrees = (angle_deg * WHEEL_DISTANCE_M) / (2 * WHEEL_RADIUS_M)
    rotate(motor1, motor2, speed, degrees, brake, block)

def rotate(motor1, motor2, speed, degrees, brake, block):
        motor1.on_for_degrees(speed=speed, degrees=degrees, brake=brake, block=False)
        motor2.on_for_degrees(speed=speed, degrees=-degrees, brake=brake, block=block)

def moveForward(motor1, motor2, speed, degrees, brake, block):
        motor1.on_for_degrees(speed=speed, degrees=degrees, brake=brake, block=False)
        motor2.on_for_degrees(speed=speed, degrees=degrees, brake=brake, block=block)

def main():# Valor inicial (color actual)

    valor_inicial = sensor.reflected_light_intensity

    print("Valor inicial:", valor_inicial)
    print("Comenzando a printear...")


    #First step: Finding Black line
    rotate_robot_degrees(large_motor_B, large_motor_C,10,0, False, True)
    while True:
        valor = sensor.reflected_light_intensity
        # print(valor)
        sth = distance_sensor.distance_centimeters
        # print(sth)
        # Si hay un cambio fuerte respecto al valor inicial → deja de imprimir
        if 0 <= valor < 20:   # Ajusta la sensibilidad
            print("Cambio detectado dejo de printear")
            #rotate(large_motor_B, large_motor_C, grados180)
            moveForward(large_motor_B, large_motor_C,10, 0, True, True)
            break

        move_cm(large_motor_B, large_motor_C,10,forward, True, True)

    leds = Leds()
    leds.set_color('LEFT', 'GREEN')
    #Second step: Detecting the correct degrees
    FORWARD_SECOND_STEP = 1000
    ROTATION_DEGREES = 200
    moveForward(large_motor_B, large_motor_C,10,FORWARD_SECOND_STEP, True, True)
    print("i moved")
    right = False
    rotate(large_motor_B, large_motor_C,10,-ROTATION_DEGREES*2, False, True)
    rotate(large_motor_B, large_motor_C,10,ROTATION_DEGREES, True, False)
    for i in range(0,300):
            valor = sensor.reflected_light_intensity
            # print(valor)
            # Si hay un cambio fuerte respecto al valor inicial → deja de imprimir
            if 0 <= valor < 20:   # Ajusta la sensibilidad
                    print("Cambio detectado dejo de printear")
                    #rotate(large_motor_B, large_motor_C, grados180)
                    moveForward(large_motor_B, large_motor_C,10, 0, True, True)
                    right = True 
                    break
            delay(0.05)

    print("here")
    if not right:
            
            
            for i in range(0,300):
                    valor = sensor.reflected_light_intensity
                    print(valor)
                    # Si hay un cambio fuerte respecto al valor inicial → deja de imprimir
                    if 0 <= valor < 20:   # Ajusta la sensibilidad
                            print("Cambio detectado dejo de printear")
                            #rotate(large_motor_B, large_motor_C, grados180)
                            moveForward(large_motor_B, large_motor_C,10, 0, True, True) 
                            break
                    delay(0.05)


if SIMULATION:
    import time
    def delay(period):
       time.sleep(period)
    import threading
    import simulation.robot as robot
    _main_thread = threading.Thread(target=main, daemon=True)
    _main_thread.start()
    robot.start()
else:
      main()

