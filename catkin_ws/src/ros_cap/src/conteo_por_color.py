#!/usr/bin/env python


import rospy
# import rospkg
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# define range of blue color in HSV
lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
lower_red = np.array([0,110,110])
upper_red = np.array([5,255,255])
lower_yellow = np.array([20,130,130])
upper_yellow = np.array([40,255,255])


class BlobColor():

    def __init__(self):


        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber( "/duckiebot/camera_node/image/raw",Image,self._process_image) 
        self.base_pub = rospy.Publisher('imagen', Image, queue_size=1)
        self.center_pub = rospy.Publisher('centro', Point, queue_size=1)
        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        self.min_area = 10
        self.conteo=0


    def _process_image(self,img):
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
        mask = cv2.inRange(image, lower_red, upper_red)
        mask2 = cv2.inRange(image, lower_blue, upper_blue)

        # Bitwise-AND mask and original image
        

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        kernel2 = np.ones((3,3),np.uint8)
         #Operacion morfologica dilate
        img_1 = cv2.dilate(mask, kernel, iterations = 3)
        img_2=cv2.erode(img_1, kernel2, iterations = 2)
        img_out = cv2.dilate(img_2, kernel, iterations = 3)
        
        
       
        segment_image = cv2.bitwise_and(frame,frame, mask=img_out
)

        image, contours, hierarchy = cv2.findContours(img_out,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            #Obtener rectangulo
            x,y,w,h = cv2.boundingRect(cnt)
                    
            #Filtrar por area minima
            if w*h > self.min_area:
                
                #Dibujar un rectangulo en la imagen
                cv2.rectangle(segment_image, (x,y), (x+w,y+h), (55,55,55), 2)  
                self.conteo=self.conteo+1
        rospy.loginfo(self.conteo)   #mostrar la cantidad de rectangulos encontrados
        self.conteo=0          #resetearlo a 0 para que no se vaya a infinito
        #Publicar frame
        msg=Image()
        msg=self.bridge.cv2_to_imgmsg(segment_image,"bgr8")
        
        self.base_pub.publish(msg)
        
  
def main():

    rospy.init_node('BlobColor')

    BlobColor()

    rospy.spin()

if __name__ == '__main__':
    main()
