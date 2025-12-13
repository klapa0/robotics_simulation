from simulation import robot
import time

tests = [
    (0.1,0.8, -30),
    (0.1, 0.1, 45),
    (0.1,0.9, -45),
    
]

def test(function_given,sleep_time):

    for test in tests:
        robot.robot.x_m = test[0]
        robot.robot.y_m = test[1]
        robot.robot.set_robot_orientation_degrees(test[2])
        function_given()
        time.sleep(sleep_time)
