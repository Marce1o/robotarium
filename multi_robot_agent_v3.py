import time
from robomaster import robot, conn, camera
import rospy
from geometry_msgs.msg import Quaternion
from geometry.msgs.msg import Point
from std_msgs.msg import String
import threading
import cv2
import yaml 

from io import StringIO
import sys

tmp = sys.stdout
resultado = StringIO()
sys.stdout = resultado
conn.scan_robot_ip_list(timeout=10)
sys.stdout = tmp
conectados = resultado.getvalue()
listaDeConectados = conectados.split("\n")
del listaDeConectados[-1]

for i in range(0,len(listaDeConectados)):
    listaDeConectados[i] = listaDeConectados[i][14:28]


with open('robot_config.yaml','r') as file:
    robot_config = yaml.safe_load(file)
    robomaster_config = robot_config['robomaster']



################ Declaracion SN de los robots
#tempSN = ["3JKCK6U0030A54","3JKCK1600303QV","3JKCK6U0030AC4","3JKCK1600303EM","3JKCK1600302LH","3JKCK1600303P6"]
SN = []

robot_names = []

################ Añadir SN que se encuentran conectadas 
for i in range(0,len(robomaster_config)):
    name = f'rm_{i}'
    g_name = f'gimbal_{i}'
    if robomaster_config[name]['SN'] in listaDeConectados:
        SN.append(robomaster_config[name]['SN'])
        robot_names.append(name)

print(SN)
print(robot_names)

control = []
robots = []
robots_data= []

blaster_data = []
state = " "

pitch_angle = 0
yaw_angle  = 0

robot_speeds = {}
gimbal_speeds = {}
gimbal_control = {}



#################################################

def cameraProcessing(connected_robots): 
        cameras = []
        for i in range(0,len(connected_robots),1):
                cameras.append(connected_robots[i].camera)
                cameras[i].start_video_stream(display=False, resolution=camera.STREAM_360P)

        while True: 
                for i in range(0,len(connected_robots),1): 
                        try: 
                                start = time.time()
                                img = cameras[i].read_cv2_image()
                                #dimensions = img.shape

                                #cv2.imwrite(f"robo{i}_img.png", img)
                                cv2.imshow(f"robo_window_{i}",img)
                                #print(f"Image dimensions of {i}: {dimensions}")

                                end = time.time()

                                time_elapsed = end - start 
                                #print(f"Time elapsed: {time_elapsed}")

                                cv2.waitKey(1)
                        
                        except Exception as e: 
                                print(e)

#################################################


################### ROS FUNCTIONS
def robotPublisher(names):
       #print('publisher')
       rospy.init_node('robot_controller',anonymous=True)
       pub = rospy.Publisher('robot_names', String, queue_size=10)
       n_robots = "".join(names)
       print(n_robots)
       rate = rospy.Rate(10)
       counter = 0
       while not rospy.is_shutdown():
                counter = counter + 1 
                rospy.loginfo(n_robots)
                pub.publish(n_robots)
                rate.sleep()

                if counter == 10:
                        break

def stateCollector(msg):
        global state
        state = msg.data 
###################  GIMBAL FUNCTIONS
def gimbal_thread(connected_robots,robot_names):
        global pitch_angle,yaw_angle
        global responseCounter
        responseCounter = 0
        meanResponseTime = 0.05
        currentMobileSleep = 0.1
        responseRate = 0
        while True:
                try:
                        for robot_num in range(0,len(connected_robots),1):
                                value = connected_robots[robot_num].gimbal.sub_angle(freq=50, callback=get_attitude)
                                
                                time.sleep(currentMobileSleep)
                                meanResponseTime = currentMobileSleep/responseCounter
                                if(meanResponseTime>=0.025 and responseCounter<=4):
                                    currentMobileSleep += 0.02
                                if(meanResponseTime<=0.02 and responseCounter>=6):
                                    currentMobileSleep -= 0.02
                                responseCounter = 0
                                #print(f"-------------,{meanResponseTime}, {currentMobileSleep}")
                                connected_robots[i].gimbal.unsub_angle()
                                                               

                                gimbal_publisher(f'{robot_names[i]}_gimbal',pitch_angle,yaw_angle)

                except Exception as e: 
                        print(e)
                        
def gimbal_publisher(topic,pitch,yaw): 
        pub = rospy.Publisher(topic, Point, queue_size=10)
        rate = rospy.Rate(10)
        counter = 0
        while not rospy.is_shutdown():
                counter = counter + 1 
                msg = Point()
                msg.x = pitch
                msg.y =  yaw
                pub.publish(msg)
                rate.sleep()

                if counter == 2:
                        break

def get_attitude(angle_info):
    global pitch_angle, yaw_angle, responseCounter
    responseCounter = responseCounter + 1
    pitch_angle,yaw_angle,pitch_ground_angle, yaw_ground_angle = angle_info

    #print(f"this is {pitch_angle}, {yaw_angle}, {responseCounter}")

def robot_assign(msg):
        global name,robot_speeds
        robot_speeds[name] = [msg.x,msg.y,msg.z,msg.w]

def gimbal_assign(msg):
       global name,gimbal_control 

       gimbal_control[name] = [msg.x,msg.y]

def robot_info(robots_names): 
        global name
        for robot in robots_names: 
                name = robot
                rospy.Subscriber(f"{robot}/wheels",Quaternion ,robot_assign)
                rospy.Subscriber(f'{robot}/gimbal_speed',Point,gimbal_assign)


#################################################
def main():
        global state, robot_speeds,gimbal_control,gimbal_speeds

        ################ Inicializacion de los robots 
        for i in range(0,len(SN),1):
                print(f"intentando {SN[i]}")
                robots.append(robot.Robot())
                robots[i].initialize(conn_type="sta",sn=SN[i])
                robot_speeds[robot_names[i]] = [0,0,0,0]
                gimbal_control[robot_names[i]] = [0,0]
                print(f"robot con sn {SN[i]} inicializado")
                print(i, robots)

        cameraThread = threading.Thread(target = cameraProcessing, args=(robots,))
        cameraThread.start()
        
        gimbalThread = threading.Thread(target = gimbal_thread, args=(robots,robot_names))
        gimbalThread.start()

        while True:
                robot_info(robot_names)

                ################ Ciclo encargado de asignar los valores correspondientes de velocidad a cada robot
                
                for i in range(0,len(robots)):

                        params = robot_speeds[robot_names[i]]
                        robots[i].chassis.drive_wheels(params[0],params[1],params[2],params[3])

                        g_params = gimbal_control[robot_names[i]]
                        robots[i].gimbal.drive_speed(g_params[0],g_params[1])
                        
                if state == 'shutdown':
                        for i in range(0,len(robots),1):
                                robots[i].chassis.drive_wheels(0,0,0,0)
                                robots[i].gimbal.drive_speed(0,0)
                        break

         ################ Cierra la conexion con cada uno de los robots 
        for i in range(1,len(robots),1):
                robots[i-1].close()

                
if __name__ == '__main__':
        robotPublisher(robot_names)
        print("done publishing")
        main()