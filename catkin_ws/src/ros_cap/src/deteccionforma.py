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
umbral_minimo=125
umbral_maximo=250


class BlobColor():

    def __init__(self):


        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber('duckiebot/camera_node/image/raw', Image, self.process_image)

        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        #Publicadores
        self.pub=rospy.Publisher('/duckiebot/camera_node/raw_camera_deteccion_amarillo', Image, queue_size=1)
        self.pub2=rospy.Publisher('/duckiebot/camera_node/raw_camera_punto', Point, queue_size=1)
        self.pubgiro= rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        self.pubpunto=rospy.Publisher('/duckiebot/geometry_msgs/posicionciudadano', Point, queue_size=1)
        
        self.pubgray1=rospy.Publisher('/duckiebot/camera_node/raw_camera_imagengray1', Image, queue_size=1)
        self.pubgray=rospy.Publisher('/duckiebot/camera_node/raw_camera_imagengray', Image, queue_size=1)
        self.pubcontornos=rospy.Publisher('/duckiebot/camera_node/raw_camera_imagencontornos', Image, queue_size=1)

        self.min_area=200



    def process_image(self,img):

        #Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        #Se deja en frame la imagen actual
        frameentero= self.cv_image
        #Restringimos el area de la imagen que procesamos
        frame=frameentero[:160,:,:]

        #Ocupamos solo la capa verde del frame
        framerojo=frame[:,:,1]
        equa=cv2.equalizeHist(framerojo)
        
        # Aplicamos canny a la imagen en escala de grises
        canny = cv2.Canny(equa, umbral_minimo, umbral_maximo)

        # Bitwise-AND mask and original image
        #segment_image = cv2.bitwise_and(frame,frame, mask= mask)

        #kernel = np.ones((5,5),np.uint8)

        #Operacion morfologica erode
        #mask1 = cv2.erode(canny, kernel, iterations = 0)
        
        #Operacion morfologica dilate
        #mask2 = cv2.dilate(canny, kernel, iterations = 3)
        
        #Transformamos el canny a BGR para usarlo en el if si lo necesitamos
        canny_out=cv2.cvtColor(canny,cv2.COLOR_GRAY2BGR)

        #transformamos equa a BGR para usarlo en el if si lo necesitamos
        cimg = cv2.cvtColor(equa,cv2.COLOR_GRAY2BGR)
        

        #Publicar imagenes
        #msg_imagenborde=self.bridge.cv2_to_imgmsg(canny_out, "bgr8")
        msg_imagengray=self.bridge.cv2_to_imgmsg(equa, "mono8")
        msg_imagengray1=self.bridge.cv2_to_imgmsg(framerojo, "mono8")
        self.pubgray1.publish(msg_imagengray1)
        self.pubgray.publish(msg_imagengray)

        dismin=20 #distancia minima entre circulos
        circles = cv2.HoughCircles(equa,cv2.HOUGH_GRADIENT,1,dismin,param1=250,param2=35,minRadius=10,maxRadius=45)

        if not circles == None:
            circles = np.uint16(np.around(circles))
            k=0
            #circles = np.uint16(circles)
            for i in circles[0,:]:
                k+=1
                # draw the outer circle
                cv2.circle(frameentero,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(frameentero,(i[0],i[1]),2,(0,0,255),3)
            print k
        else:
            print 0
        
        msg_imagencontornos=self.bridge.cv2_to_imgmsg(frameentero, "bgr8")
        self.pubcontornos.publish(msg_imagencontornos)

       
def main():

    rospy.init_node('BlobColor')

    BlobColor()

    rospy.spin()

if __name__ == '__main__':
    main()

