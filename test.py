from simulation import robot
import time

tests = [
    
    (0.1, 0.3, 45),
    (0.1,-0.3, -45),
    (0.1,0.4, 30),
    
]

def test(function_given,sleep_time):

    for test in tests:
        robot.robot.reset()
        robot.robot.x_m = test[0]
        robot.robot.set_position_m_from_line(test[1])
        robot.robot.set_robot_orientation_degrees(test[2])
        function_given()
        time.sleep(sleep_time)
