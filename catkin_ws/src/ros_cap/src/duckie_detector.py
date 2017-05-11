#!/usr/bin/env python


import rospy
# import rospkg
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# define range of blue color in HSV
lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
lower_red = np.array([0,50,50])
upper_red = np.array([10,255,255])
lower_yellow = np.array([20,130,130])
upper_yellow = np.array([40,255,255])
#Variables globales del rectangulo:
xrect = 0
yrect = 0
wrect = 0
hrect = 0

class duckiedetector():

    def __init__(self):

        
        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber( "/duckiebot/camera_node/image/rect",Image,self._process_image) 
        self.base_pub = rospy.Publisher('imagen', Image, queue_size=1)
        self.distance_3d = rospy.Publisher('distancia3d', Point, queue_size=1)
        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        self.min_area = 100
        self.max_area = 0



    def _process_image(self,img):
        global xrect
        global yrect
        global wrect
        global hrect
        #Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
        #Se deja en frame la imagen actual
        frame = self.cv_image

        #Cambiar tipo de color de BGR a HSV
        image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Filtrar colores de la imagen en el rango utilizando 
        mask = cv2.inRange(image, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        segment_image = cv2.bitwise_and(frame,frame, mask= mask)


        kernel = np.ones((5,5),np.uint8)

        #Operacion morfologica erode
        img_out = cv2.erode(mask, kernel, iterations = 3)
        
        #Operacion morfologica dilate
        img_out = cv2.dilate(img_out, kernel, iterations = 6)

        image, contours, hierarchy = cv2.findContours(img_out,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        self.max_area=0
        for cnt in contours:
            #Obtener rectangulo
            x,y,w,h = cv2.boundingRect(cnt)
            
                    
            #Filtrar por area minima
            if w*h > self.min_area:
                
                #Dibujar un rectangulo en la imagen
                cv2.rectangle(frame, (x,y), (x+w,y+h), (55,55,55), 2)
            if w*h >= self.max_area:
                self.max_area = w+h
                xrect=x
                yrect=y
                wrect=w
                hrect=h
                                

        #Publicar frame
        msg=Image()
        msg=self.bridge.cv2_to_imgmsg(frame,"bgr8")
        self.base_pub.publish(msg)
        
        #Publicar Point center de mayor tamanio
        msj=Point()
        msj.x= xrect+wrect/2
        msj.y= yrect+hrect/2
        disfoc=340.9
        altpix=hrect
        altpat=3.5
        dispat=disfoc*altpat/altpix
        msj.z=dispat
        self.distance_3d.publish(msj)

def main():

    rospy.init_node('duckie_detector')

    duckiedetector()

    rospy.spin()

if __name__ == '__main__':
    main()
