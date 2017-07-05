#!/usr/bin/env python

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
lower_red = np.array([0,50,50])
<<<<<<< HEAD
upper_red = np.array([10,255,255])
lower_yellow = np.array([20,130,130])
upper_yellow = np.array([40,255,255])
=======
upper_red = np.array([20,255,255])
lower_yellow = np.array([20,120,50])
upper_yellow = np.array([55,255,255])
>>>>>>> a6c558c2d8d02fdf31d0ed3e9489fc8e6af5233b


class SegmentImage():

    def __init__(self):


        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
<<<<<<< HEAD
        self.image_subscriber = rospy.Subscriber( "/duckiebot/camera_node/image/raw",Image,self._process_image)
        self.base_pub = rospy.Publisher('imagen', Image, queue_size=1)
         # Publish each sec
=======
        self.image_subscriber = rospy.Subscriber('duckiebot/camera_node/image/raw', Image, self.process_image)

>>>>>>> a6c558c2d8d02fdf31d0ed3e9489fc8e6af5233b
        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()
        self.pub=rospy.Publisher('/duckiebot/camera_node/raw_camera_info', Image, queue_size=1)


    def process_image(self,img):

        #Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        #Se deja en frame la imagen actual
        frame = self.cv_image

        #Cambiar tipo de color de BGR a HSV
<<<<<<< HEAD
        image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)         
        # Filtrar colores de la imagen en el rango utilizando
 
        mask = cv2.inRange(image, lower_yellow, upper_yellow)
=======
        image_out=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        
        # Filtrar colores de la imagen en el rango utilizando 
        mask = cv2.inRange(image_out, lower_yellow, upper_yellow)
>>>>>>> a6c558c2d8d02fdf31d0ed3e9489fc8e6af5233b

        # Bitwise-AND mask and original image
        segment_image = cv2.bitwise_and(frame,frame, mask= mask)

        #Publicar imagenes
<<<<<<< HEAD
        msg=Image()
        msg=self.bridge.cv2_to_imgmsg(segment_image,"bgr8")
        self.base_pub.publish(msg)
=======
        msg1= self.bridge.cv2_to_imgmsg(segment_image, "bgr8")
        self.pub.publish(msg1)
>>>>>>> a6c558c2d8d02fdf31d0ed3e9489fc8e6af5233b


def main():

    rospy.init_node('SegmentImage')

    SegmentImage()

    rospy.spin()

if __name__ == '__main__':
    main()
