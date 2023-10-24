import rospy 
import math 
import time
from geometry_msgs.msg import TransformStamped


def getGimbal(msg):
        q = msg.transform.rotation
        
        yaw = math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
        pitch = math.asin(-2.0*(q.x*q.z - q.w*q.y))
        roll = math.atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)

        #print(yaw,pitch,roll)
        time.sleep(0.1)

        yaw_g = math.degrees(yaw)
        pitch_g = math.degrees(pitch)
        roll_g = math.degrees(roll)

        print(yaw_g,pitch_g,roll_g)



def dataCollector():
        rospy.init_node('robot_controller',anonymous=True)
        rospy.Subscriber("/vicon/Gun1/Gun1",TransformStamped,getGimbal)
        rospy.spin()


dataCollector()