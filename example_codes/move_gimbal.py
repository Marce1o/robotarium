import time
import robomaster
from robomaster import robot


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")

    ep_gimbal = ep_robot.gimbal

    #ep_gimbal.moveto(pitch=180, yaw=0, pitch_speed=100, yaw_speed=100).wait_for_completed()

    # slp =3
    # # 控制云台向右以30度每秒旋转3秒
    ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
    time.sleep(3)

    # # 控制云台向左以30度每秒旋转3秒
    # ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=-30)
    # time.sleep(slp)

    # # 控制云台向上以30度每秒旋转3秒
    # ep_gimbal.drive_speed(pitch_speed=10, yaw_speed=0)
    # time.sleep(slp)

    # # 控制云台向下以30度每秒旋转3秒
    # ep_gimbal.drive_speed(pitch_speed=-10, yaw_speed=0)
    # time.sleep(slp)

    #ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=100, yaw_speed=100).wait_for_completed()

    ep_robot.close()