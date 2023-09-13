import time
from robomaster import robot
from robomaster import led


if __name__ == '__main__':
    print("moving arm")
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    
    
    ep_arm = ep_robot.robotic_arm

    ep_arm.move(40,40)

    time.sleep(10)

    ep_arm.recenter()

    ep_robot.close()