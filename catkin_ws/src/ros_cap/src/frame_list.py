#!/usr/bin/env python


import rospy
# import rospkg
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty, EmptyResponse

from cv_bridge import CvBridge, CvBridgeError

import numpy as np


class BlobColor():

    def __init__(self):
        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.joy_subscriber = rospy.Subscriber("/duckiebot/joy", Joy, self._process_button)
        self.image_subscriber = rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self._process_image)

        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        self.min_area = 10
        self.images = []



    def _process_image(self,img):
        #Se cambiar mensage tipo ros a imagen opencv
        self.cv_image = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        #Se deja en frame la imagen actual
        frame = self.cv_image
        self.images.append(frame)
        if len(self.images) == 31:
            self.images = self.images[1:]

    def _process_button(self, joy_msg):
        if joy_msg.buttons[0]:
            print len(self.images)
            local_images = self.images[:]
            for i, image in enumerate(local_images):
                print i, 'hola'

def main():

    rospy.init_node('BlobColor')

    BlobColor()

    rospy.spin()

if __name__ == '__main__':
    main()

