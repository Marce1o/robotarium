from robomaster import robot
import time



if __name__ == '__main__':

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")


    ep_robot.robotic_arm.recenter().wait_for_completed()


    time.sleep(5)

    print("now moving arm")
    ep_robot.robotic_arm.move(x=80,y=100).wait_for_completed()

    ep_robot.close()