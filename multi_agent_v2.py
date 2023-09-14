import time
from robomaster import robot, conn
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# 3JKCK1600303EM


print(conn.scan_robot_ip("3JKCK6U0030A59",timeout=10))

tempSN = ["3JKCK6U0030A54","3JKCK1600303QV","3JKCK6U0030AC4","3JKCK1600303EM"]
SN = []
for i in range(0,len(tempSN),1):
        if(conn.scan_robot_ip(tempSN[i],timeout=10) != None):
                SN.append(tempSN[i])

print(SN)




control = []
robot1 = [0,0,0,0]
robot2 = [0,0,0,0]
control.append(robot1)
control.append(robot2)

robots = []

state = " "

def robot1_Collector(msg): 
    global robot1

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

        for i in range(0,len(SN),1):
                print(f"intentando {SN[i]}")
                robots.append(robot.Robot())
                robots[i].initialize(conn_type="sta",sn=SN[i])
                print(f"robot con sn {SN[i]} inicializado")
                print(i, robots)


        while True:
               dataCollector()
               for i in range(0,len(robots),1):
                        params = []
                        for j in range(0,4,1):
                                params.append(control[i][j])
                        
                        robots[i].chassis.drive_wheels(params[0],params[1],params[2],params[3])
               

               if state == 'shutdown':
                        for i in range(0,len(robots),1):
                                robots[i].chassis.drive_wheels(0,0,0,0)
                        break
        
        for i in range(1,len(robots),1):
                robots[i-1].close()

                
if __name__ == '__main__':
        main()
