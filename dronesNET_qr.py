
import time
from robomaster import conn
from MyQR import myqr
from PIL import Image


QRCODE_NAME = "qrcode.png"

if __name__ == '__main__':

    helper = conn.ConnectionHelper()
    #info = helper.build_qrcode_string(ssid="drones", password="dronesLab")
    #info = helper.build_qrcode_string(ssid="red2", password="224426628816")
    info = helper.build_qrcode_string(ssid="Moto G8 plus", password="123456789")
    #info = helper.build_qrcode_string(ssid="Lab_Robotica", password="coke.fanta.sprite")
    myqr.run(words=info)
    time.sleep(1)
    img = Image.open(QRCODE_NAME)   
    img.show()
    if helper.wait_for_connection():
        print("Connected!")
    else:
        print("Connect failed!")
