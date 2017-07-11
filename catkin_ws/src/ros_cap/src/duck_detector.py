#!/usr/bin/env python


import rospy
# import rospkg
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
import numpy as np

# define range of blue color in HSV
lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
lower_red = np.array([0,50,50])
upper_red = np.array([20,255,255])
lower_yellow = np.array([20,180,50])
upper_yellow = np.array([30,255,255])


class BlobColor():

    def __init__(self):


        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber('duckiebot/camera_node/image/rect', Image, self.process_image)

 
        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        #Publicadores
        self.pub=rospy.Publisher('/duckiebot/camera_node/raw_camera_deteccion_amarillo', Image, queue_size=1)
        self.pub2=rospy.Publisher('/duckiebot/camera_node/raw_camera_punto', Point, queue_size=1)
        self.pubgiro= rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)

        self.pubpunto=rospy.Publisher('/duckiebot/geometry_msgs/posicionciudadano', Point, queue_size=1)
       

        self.min_area=200



    def process_image(self,img):

        #Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        #Se deja en frame la imagen actual
        frame = self.cv_image

        #Cambiar tipo de color de BGR a HSV
        image_out=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        
        # Filtrar colores de la imagen en el rango utilizando 
        mask = cv2.inRange(image_out, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        #segment_image = cv2.bitwise_and(frame,frame, mask= mask)

        kernel = np.ones((5,5),np.uint8)

        #Operacion morfologica erode
        mask1 = cv2.erode(mask, kernel, iterations = 2)
        
        #Operacion morfologica dilate
        mask2 = cv2.dilate(mask1, kernel, iterations = 2)

        image, contours, hierarchy = cv2.findContours(mask2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        xx=0
        ww=0
        yy=0
        hh=0
	area_maxima=0
	for cnt in contours:
		#Obtener rectangulo
		x,y,w,h = cv2.boundingRect(cnt)
		area=w*h
		if area > area_maxima:
			areamax=area
			xx=x
			ww=w
			yy=y
			hh=h
		#Filtrar por area minima
		if w*h > self.min_area:

			 #Dibujar un rectangulo en la imagen
			cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,0), 2)

        #Publicar frame
        msg_imagen=self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.pub.publish(msg_imagen)
        #Publicar Point center de mayor tamanio
        centrox=xx+ww/2
        centroy=yy+hh/2
        centroxcamara= msg_imagen.width/2
        centroycamara= msg_imagen.height/2
        msgpunto=Point()
        fx=212.42692381213064
        fy=212.78513200903114
        cx=163.8421102477024
        cy=141.4663252556416
        #L1=(fx*3.2)/ww 
        #L2=(fy*4)/hh
	if ww == 0:
		z=10000000000
	else:
		z=(fx*3.2)/ww

        msgpunto.z=z
        msgpunto.x=centrox
        msgpunto.y=centroy
        self.pubpunto.publish(msgpunto)
        

def main():

    rospy.init_node('BlobColor')

    BlobColor()

    rospy.spin()

if __name__ == '__main__':
    main()
