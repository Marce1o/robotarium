import time
from robomaster import robot, conn, camera
import rospy
from geometry_msgs.msg import Quaternion
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

def blaster_data_collector(msg):
        global blaster_data
        blaster_data = msg.data.split('~')

        temp=blaster_data
        blaster_data = []
        for i in range(0,len(temp),1):
                data = temp[i].split(",")
                toappend = []
                for j in range(0,len(data),1):
                        toappend.append(float(data[j]))
                blaster_data.append(toappend)

def stateCollector(msg):
        global state
        state = msg.data 

def dataCollector():
        rospy.Subscriber("state",String,stateCollector)
        rospy.Subscriber("robots_data",String,robots_data_collector)
        rospy.Subscriber('blaster',String,blaster_data_collector)
        rospy.sleep(0.2)

def robotDataCollector(name): 
        rospy.Subscriber(f'{name}/wheel_speed')

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

###################  GIMBAL FUNCTIONS
def gimbal_thread(connected_robots):
        global pitch_angle,yaw_angle
        global responseCounter
        responseCounter = 0
        meanResponseTime = 0.05
        currentMobileSleep = 0.1
        responseRate = 0
        gimbal_state = ""
        while True:
                gimbal_state = ""
                try: 
                        for i in range(0,len(connected_robots),1):
                                robot_num = i
                                value = connected_robots[i].gimbal.sub_angle(freq=50, callback=get_attitude)
                                if i != len(connected_robots):
                                        tempstr = (str(pitch_angle) + "," + str(yaw_angle)+ "~")
                                else:
                                        tempstr = (str(pitch_angle) + "," + str(yaw_angle))

                                gimbal_state = gimbal_state + tempstr
                                

                                time.sleep(currentMobileSleep)
                                meanResponseTime = currentMobileSleep/responseCounter
                                if(meanResponseTime>=0.025 and responseCounter<=4):
                                    currentMobileSleep += 0.02
                                if(meanResponseTime<=0.02 and responseCounter>=6):
                                    currentMobileSleep -= 0.02
                                responseCounter = 0
                                #print(f"-------------,{meanResponseTime}, {currentMobileSleep}")
                                connected_robots[i].gimbal.unsub_angle()
                                                               

                                gimbal_publisher(gimbal_state)

                except Exception as e: 
                        print(e)
                        
def gimbal_publisher(gimbal_state): 
        pub = rospy.Publisher('gimbal_state', String, queue_size=10)
        rate = rospy.Rate(10)
        counter = 0
        while not rospy.is_shutdown():
                counter = counter + 1 
                #rospy.loginfo(gimbal_state)
                pub.publish(gimbal_state)
                rate.sleep()

                if counter == 2:
                        break

def get_attitude(angle_info):

    global pitch_angle, yaw_angle, responseCounter
    responseCounter = responseCounter + 1
    pitch_angle,yaw_angle,pitch_ground_angle, yaw_ground_angle = angle_info

    #print(f"this is {pitch_angle}, {yaw_angle}, {responseCounter}")

def robot_asign(msg):
        global name,robot_speeds
        robot_speeds[name] = [msg.x,msg.y,msg.z,msg.w]

def robot_info(robots_names): 
        global name
        for robot in robots_names: 
                name = robot
                rospy.Subscriber(f"{robot}/wheels",Quaternion ,robot_asign)

#################################################
def main():
        global state, robot_speeds

        ################ Inicializacion de los robots 
        for i in range(0,len(SN),1):
                print(f"intentando {SN[i]}")
                robots.append(robot.Robot())
                robots[i].initialize(conn_type="sta",sn=SN[i])
                robot_speeds[robot_names[i]] = [0,0,0,0]
                print(f"robot con sn {SN[i]} inicializado")
                print(i, robots)

        cameraThread = threading.Thread(target = cameraProcessing, args=(robots,))
        cameraThread.start()
        
        #gimbalThread = threading.Thread(target = gimbal_thread, args=(robots,))
        #gimbalThread.start()

        while True:
                #dataCollector()

                robot_info(robot_names)

                gimbal_state = ''

                ################ Ciclo encargado de asignar los valores correspondientes de velocidad a cada robot
                
                for i in range(0,len(robots)):

                        params = robot_speeds[robot_names[i]]
                        robots[i].chassis.drive_wheels(params[0],params[1],params[2],params[3])
                        # params = []
                        # for j in range(0,4,1):
                        #                 try:
                        #                         params.append(robots_data[i][j])
                        #                 except Exception as e:
                        #                         pass
                        #                         #print(e)
                        # try:
                        #         robots[i].chassis.drive_wheels(params[0],params[1],params[2],params[3])
                        # except Exception as e:
                        #         pass
                        #         #print(e)
                        
                        #params = []
                        # for j in range(0,2,1):
                        #                 try:
                        #                         params.append(blaster_data[i][j])
                        #                 except Exception as e:
                        #                         pass
                        #                         #print(e)

                        # try:
                        #         robots[i].gimbal.drive_speed(params[0],params[1])
                        # except Exception as e:
                        #         #print(e)
                        #         pass
                        
                        # pitch = 0 
                        # yaw = 0 


                if state == 'shutdown':
                        for i in range(0,len(robots),1):
                                robots[i].chassis.drive_wheels(0,0,0,0)
                        break

         ################ Cierra la conexion con cada uno de los robots 
        for i in range(1,len(robots),1):
                robots[i-1].close()

                
if __name__ == '__main__':
        robotPublisher(robot_names)
        print("done publishing")
        main()
