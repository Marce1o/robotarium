import time
from robomaster import robot
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


SN = ["3JKCK1600303EM","3JKCK6U0030A54"]

robot1 = [0,0,0,0]
robot2 = [0,0,0,0]



state = " "

def robot1_Collector(msg): 
    global robot_1

    robot1[0] = msg.linear.x
    robot1[1] = msg.linear.y
    robot1[2] = msg.linear.z
    robot1[3] = msg.angular.x

def robot2_Collector(msg): 
    global robot2
    robot2[0] = msg.linear.x
    robot2[1] = msg.linear.y
    robot2[2] = msg.linear.z
    robot2[3] = msg.angular.x



def stateCollector(msg):
        global state
        state = msg.data 

def dataCollector():
        rospy.init_node('robot_controller',anonymous=True)
        rospy.Subscriber("robot_1",Twist,robot1_Collector)
        rospy.Subscriber("robot_2",Twist,robot2_Collector)
        rospy.Subscriber("state",String,stateCollector)

        rospy.sleep(0.2)

def main():
        global state,robot1,robot2
        counter = 0
        robot_1 = robot.Robot()
        robot_1.initialize(conn_type="sta",sn=SN[counter])
        chassis_robot_1= robot_1.chassis
        counter = counter + 1
        robot_2 = robot.Robot()
        robot_2.initialize(conn_type="sta",sn=SN[counter])
        chassis_robot_2= robot_2.chassis


        while True:
               dataCollector()
               chassis_robot_1.drive_wheels(robot1[0],robot1[1],robot1[2],robot1[3])
               chassis_robot_2.drive_wheels(robot2[0],robot2[1],robot2[2],robot2[3])

               if state == 'shutdown':
                    chassis_robot_1.drive_wheels(0,0,0,0)
                    chassis_robot_2.drive_wheels(0,0,0,0)
                    break
        
        robot_1.close()
        robot_2.close()

                
if __name__ == '__main__':
        main()
