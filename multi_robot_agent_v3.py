import time
from robomaster import robot, conn, camera
import rospy
from geometry_msgs.msg import Quaternion,Point
from std_msgs.msg import String
import threading
import cv2
import yaml 
from io import StringIO
import sys

#--------------------------------------- BUSCAR ROBOTS EN LA RED

""" La libreria sys y la stringio nos permiten extraer como string lo que se manda en la terminal, 
mientras que la funcion scan_robot_ip_list nos permite extraer que robots estan conectados a la 
red, devolviendo una IP y un numero serial, extraemos eso como string que añadimos a una lista 
y procedemos a limpiar y dejando solo el SN  """

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

#--------------------------------------- CONFIGURAR POSIBLES CONEXIONES

""" En esta seccion abrimos el archivo de configuracion de robots lo que nos permite extraer las SN 
de los robots posiblemente conectados, los cuales son comparados con la lista generada previamente
para obtener aquellos que estan en la red actualmente, los cuales se hacen append a una nueva lista
llamada SN, tambien se guarda el nombre de cada robot como parte de una lista más  """


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


#--------------------------------------- Declaracion de Variables

robots = [] #Lista que guarda los robots inicializados

robot_speeds = {} #Diccionario donde se guardan las velocidades de  las llantas de cada robot iniciado
gimbal_control = {}#Diccionario donde se guardan las señales de control de cada gimbal iniciado

exec_state = 'run' #variable de estado de ejecución

#--------------------------------------- FUNCIONES 


#---------------- CAMARA
def cameraProcessing(connected_robots): 
        global exec_state
        cameras = []
        for i in range(0,len(connected_robots),1):
                cameras.append(connected_robots[i].camera)
                cameras[i].start_video_stream(display=False, resolution=camera.STREAM_360P)

        while exec_state != 'shutdown': 
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
        
        print('Killed Camera')

#---------------- FUNCIONES DE ROS

def robotPublisher(names):
       """ Funcion que se encarga de publicar como string la lista de nombres de los robots 
       encontrados en la red  """

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

        """ funcion que se encarga de asignar el valor del estado de ejecución que publica matlab """
        global exec_state
        exec_state = msg.data 

def getState():
        """ Funcion que se encarga de suscribirse al topico de estado de ejecución """
        while exec_state != 'shutdown':
                rospy.Subscriber('exec_state',String,stateCollector)

        print('killed exec state')


def robot_assign(msg):

        """ Funcion que asigna a cada robot su correspondiente velocidad dentro del diccionario """

        global name,robot_speeds
        robot_speeds[name] = [msg.x,msg.y,msg.z,msg.w]

def gimbal_assign(msg):
       """ Funcion que asigna a cada gimbal su correspondiente velocidad dentro del diccionario """

       global name,gimbal_control 

       gimbal_control[name] = [msg.x,msg.y]

def robot_info(robots_names): 
        """ Funcion que se suscribe a cada uno de los topicos del robot"""

        global name
        for robot in robots_names: 
                name = robot
                rospy.Subscriber(f"{robot}/wheels",Quaternion ,robot_assign)
                rospy.Subscriber(f'{robot}/gimbal_speed',Point,gimbal_assign)
                #rospy.Subscriber('exec_state',String,stateCollector)


#################################################
def main():
        global exec_state, robot_speeds,gimbal_control

        #------------------ INICIALIZA CADA ROBOT
        for i in range(0,len(SN),1):
                print(f"intentando {SN[i]}")
                robots.append(robot.Robot())
                robots[i].initialize(conn_type="sta",sn=SN[i])
                robot_speeds[robot_names[i]] = [0,0,0,0]
                gimbal_control[robot_names[i]] = [0,0]
                print(f"robot con sn {SN[i]} inicializado")
                print(i, robots)


        #------------------ INICIALIZA THREAD PARA LEER CAMARAS Y ESTADO DE EJECUCION
        cameraThread = threading.Thread(target = cameraProcessing, args=(robots,))
        cameraThread.start()
        
        executionThread = threading.Thread(target=getState)
        executionThread.start()

        while exec_state == 'run':

                """durante este proceso el programa se comuncia con cada uno de los robots y les da 
                sus velocidades de chasis y gimbal especificas, siempre y cuando el valor de 
                estado de ejecución sea igual a run, si este valor es diferente detiene los robots y 
                cierra la comunicación con los dispositvos"""

                robot_info(robot_names)

                for i in range(0,len(robots)):

                        params = robot_speeds[robot_names[i]]
                        robots[i].chassis.drive_wheels(params[0],params[1],params[2],params[3])

                        g_params = gimbal_control[robot_names[i]]
                        robots[i].gimbal.drive_speed(g_params[0],g_params[1])
                        
        
        if exec_state == 'shutdown':
                for i in range(0,len(robots)):

                        robots[i].chassis.drive_wheels(0,0,0,0)

                        print('Killed chassis')
                
                for i in range(0,len(robots)):
                        robots[i].gimbal.recenter(60,60).wait_for_completed()
                        robots[i].gimbal.drive_speed(0,0)

                        robots[i].close()

                        print('Killed robots')
                
if __name__ == '__main__':
        robotPublisher(robot_names)
        print("done publishing")
        main()
