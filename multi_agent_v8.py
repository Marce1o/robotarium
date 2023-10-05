import time
from robomaster import robot, conn, camera
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int16
import threading
import cv2

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

################ Declaracion SN de los robots
tempSN = ["3JKCK6U0030A54","3JKCK1600303QV","3JKCK6U0030AC4","3JKCK1600303EM","3JKCK1600302LH","3JKCK1600303P6"]
SN = []

################ Añadir SN que se encuentran conectadas 
for i in range(0,len(tempSN)):
    if tempSN[i] in listaDeConectados:
        SN.append(tempSN[i])

print(SN)

control = []
robots = []
robots_data= []


blaster_data = []
state = " "

def robotPublisher(SN):
       #print('publisher')
       rospy.init_node('robot_controller',anonymous=True)
       pub = rospy.Publisher('robots', Int16, queue_size=10)
       n_robots = len(SN)
       rate = rospy.Rate(10)
       counter = 0
       while not rospy.is_shutdown():
                counter = counter + 1 
                rospy.loginfo(n_robots)
                pub.publish(n_robots)
                rate.sleep()

                if counter == 10:
                        break

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


################ Function que configura los topics de ros a leer 
def dataCollector():
        rospy.init_node('robot_controller',anonymous=True)
        rospy.Subscriber("state",String,stateCollector)
        rospy.Subscriber("robots_data",String,robots_data_collector)
        rospy.Subscriber('blaster',String,blaster_data_collector)
        rospy.sleep(0.2)

def cameraProcessing(connected_robots):
    
    while True:
        for i in range(0,len(connected_robots),1):
                ep_camera = connected_robots[i].camera

                try: 
                        ep_camera.start_video_stream(display=True, resolution=camera.STREAM_720P)
                        time.sleep(0.1)
                        img = ep_camera.read_cv2_image()
                        ep_camera.stop_video_stream()
                        dimensions = img.shape

                        cv2.imwrite(f"robo{i}_img.png", img)
                        print(f"Image dimensions of {i}: {dimensions}")

                        
                except Exception as e: 
                       print(e)

def camera_robomaster(robot,index):
    ep_camera = robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    while True:
           
        ret,frame = ep_camera.read_cv2_image()

        cv2.imshow(f'robot_{index}',frame)

        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break
  
    # After the loop release the cap object 

    ep_camera.release() 
    ep_camera.stop_video_stream()
    # Destroy all the windows 
    cv2.destroyAllWindows() 
                        
        

def main():
        global state

        camera_thread = []

        ################ Inicializacion de los robots 
        for i in range(0,len(SN),1):
                print(f"intentando {SN[i]}")
                robots.append(robot.Robot())
                robots[i].initialize(conn_type="sta",sn=SN[i])
                print(f"robot con sn {SN[i]} inicializado")
                print(i, robots)


        for i in range(0,len(robots)):
                camera_thread.append(threading.Thread(target = camera_robomaster, args=(robot[i],i,)))

                camera_thread[i].start()

        while True:
                dataCollector()

                ################ Ciclo encargado de asignar los valores correspondientes de velocidad a cada robot
                for i in range(0,len(robots)):
                    params = []
                    for j in range(0,4,1):
                                try:
                                        params.append(robots_data[i][j])
                                except Exception as e:
                                        print(e)
                    try:
                           robots[i].chassis.drive_wheels(params[0],params[1],params[2],params[3])
                    except Exception as e:
                            print(e)
                    
                    params = []
                    for j in range(0,2,1):
                                try:
                                        params.append(blaster_data[i][j])
                                except Exception as e:
                                        print(e)
                    try:
                           robots[i].gimbal.drive_speed(params[0],params[1])
                    except Exception as e:
                            print(e)


                if state == 'shutdown':
                        for i in range(0,len(robots),1):
                                robots[i].chassis.drive_wheels(0,0,0,0)
                        break
               
               
         ################ Cierra la conexion con cada uno de los robots 
        for i in range(1,len(robots),1):
                robots[i-1].close()

                
if __name__ == '__main__':
        robotPublisher(SN)
        print("done publishing")
        main()
