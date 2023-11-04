import time
from robomaster import robot, conn
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# "3JKCK1600303QV","3JKCK6U0030AC4","3JKCK1600303EM"




 ################ Declaracion SN de los robots
tempSN = ["3JKCK6U0030A54","3JKCK1600303QV","3JKCK6U0030AC4","3JKCK1600303EM"]
SN = []


################ Añadir SN que se encuentran conectadas 
for i in range(0,len(tempSN),1):
        if(conn.scan_robot_ip(tempSN[i],timeout=10) != None):
                SN.append(tempSN[i])

print(SN)




control = []
robots = []
robots_data= []

state = " "


################ Funcion encargada de obtener la informacion de los robots
def robots_data_collector(msg):
        global robots_data
        robots_data = msg.data.split("~")
        temp=robots_data
        robots_data = []
        for i in range(0,len(temp),1):
                data = temp[i].split(",")
                toappend = []
                for j in range(0,len(data),1):
                        toappend.append(float(data[j]))
                robots_data.append(toappend)
        #print(robots_data)



def stateCollector(msg):
        global state
        state = msg.data 


################ Function que configura los topics de ros a leer 
def dataCollector():
        rospy.init_node('robot_controller',anonymous=True)
        rospy.Subscriber("state",String,stateCollector)
        rospy.Subscriber("robots_data",String,robots_data_collector)
        rospy.sleep(0.2)

def main():
        global state



        ################ Inicializacion de los robots 
        for i in range(0,len(SN),1):
                print(f"intentando {SN[i]}")
                robots.append(robot.Robot())
                robots[i].initialize(conn_type="sta",sn=SN[i])
                print(f"robot con sn {SN[i]} inicializado")
                print(i, robots)

        while True:
               dataCollector()


                ################ Ciclo encargado de asignar los valores correspondientes de velocidad a cada robot
               for i in range(0,len(robots),1):
                        params = []
                        for j in range(0,4,1):
                                try:
                                        params.append(robots_data[i][j])
                                except Exception as e:
                                        print(e)
                        
                        robots[i].chassis.drive_wheels(params[0],params[1],params[2],params[3])
               

               if state == 'shutdown':
                        for i in range(0,len(robots),1):
                                robots[i].chassis.drive_wheels(0,0,0,0)
                        break
               
               
         ################ Cierra la conexion con cada uno de los robots 
        for i in range(1,len(robots),1):
                robots[i-1].close()

                
if __name__ == '__main__':
        main()
