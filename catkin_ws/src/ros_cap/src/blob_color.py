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
<<<<<<< HEAD
upper_red = np.array([10,255,255])
lower_yellow = np.array([20,130,130])
upper_yellow = np.array([40,255,255])
=======
upper_red = np.array([20,255,255])
lower_yellow = np.array([20,180,50])
upper_yellow = np.array([30,255,255])
>>>>>>> a6c558c2d8d02fdf31d0ed3e9489fc8e6af5233b


class BlobColor():

    def __init__(self):


        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
<<<<<<< HEAD
        self.image_subscriber = rospy.Subscriber( "/duckiebot/camera_node/image/raw",Image,self._process_image) 
        self.base_pub = rospy.Publisher('imagen', Image, queue_size=1)
        self.center_pub = rospy.Publisher('centro', Point, queue_size=1)
=======
        self.image_subscriber = rospy.Subscriber('duckiebot/camera_node/image/raw', Image, self.process_image)

>>>>>>> a6c558c2d8d02fdf31d0ed3e9489fc8e6af5233b
        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

<<<<<<< HEAD
        self.min_area = 100
        self.max_area
=======
        #Publicadores
        self.pub=rospy.Publisher('/duckiebot/camera_node/raw_camera_deteccion_amarillo', Image, queue_size=1)
        self.pub2=rospy.Publisher('/duckiebot/camera_node/raw_camera_punto', Point, queue_size=1)
        self.pubgiro= rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        self.pubpunto=rospy.Publisher('/duckiebot/geometry_msgs/posicionciudadano', Point, queue_size=1)
        

        self.min_area=200

>>>>>>> a6c558c2d8d02fdf31d0ed3e9489fc8e6af5233b


    def process_image(self,img):

        #Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
<<<<<<< HEAD
=======

>>>>>>> a6c558c2d8d02fdf31d0ed3e9489fc8e6af5233b
        #Se deja en frame la imagen actual
        frame = self.cv_image

        #Cambiar tipo de color de BGR a HSV
<<<<<<< HEAD
        image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Filtrar colores de la imagen en el rango utilizando 
        mask = cv2.inRange(image, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        segment_image = cv2.bitwise_and(frame,frame, mask= mask)

=======
        image_out=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        
        # Filtrar colores de la imagen en el rango utilizando 
        mask = cv2.inRange(image_out, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        #segment_image = cv2.bitwise_and(frame,frame, mask= mask)
>>>>>>> a6c558c2d8d02fdf31d0ed3e9489fc8e6af5233b

        kernel = np.ones((5,5),np.uint8)

        #Operacion morfologica erode
<<<<<<< HEAD
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
                msj=Point()
                msj.x= x+w/2
                msj.y= y+h/2
                self.center_pub.publish(msj)                

        #Publicar frame
        msg=Image()
        msg=self.bridge.cv2_to_imgmsg(frame,"bgr8")
        self.base_pub.publish(msg)
   
        #Publicar Point center de mayor tamanio
        
        self.center_pub.publish(msj)
=======
        mask1 = cv2.erode(mask, kernel, iterations = 2)
        
        #Operacion morfologica dilate
        mask2 = cv2.dilate(mask1, kernel, iterations = 2)

        image, contours, hierarchy = cv2.findContours(mask2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        xx=0
        ww=0
        yy=0
        hh=0
        for cnt in contours:
                  #Obtener rectangulo
                  x,y,w,h = cv2.boundingRect(cnt)
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
        msgpunto=Point()
        msgpunto.x=centrox
        msgpunto.y=centroy
        self.pubpunto.publish(msgpunto)

        centroxcamara= msg_imagen.width/2
        deltac=centrox-centroxcamara #distancia entre centros, el objeto esta a la izquiera del centro de la camara, es negativo, de lo contrario, es positivo
        msg1 = Twist2DStamped()
        msg1.header.stamp = rospy.get_rostime()
        
        if abs(deltac)<=50:
                msg1.omega=0
        elif centrox==0:
                msg1.omega=0
        else:
                msg1.omega= -0.03*deltac

        self.pubgiro.publish(msg1)

>>>>>>> a6c558c2d8d02fdf31d0ed3e9489fc8e6af5233b
def main():

    rospy.init_node('BlobColor')

    BlobColor()

    rospy.spin()

if __name__ == '__main__':
    main()

