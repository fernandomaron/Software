#!/usr/bin/env python


import rospy
# import rospkg
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
import numpy as np

# define range of blue color in HSV
umbral_minimo=250/2
umbral_maximo=250

# define range of blue color in HSV
lower_redm = np.array([170,100,100])
upper_redm = np.array([190,255,255])
lower_red = np.array([0,110,110])
upper_red = np.array([5,255,255])
lower_yellow = np.array([20,130,130])
upper_yellow = np.array([40,255,255])


class BlobColor():

    def __init__(self):

        self.images=[]
        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber('duckiebot/camera_node/image/raw', Image, self.process_image)
        self.joy_subscriber = rospy.Subscriber('duckiebot/joy', Joy, self.process_button)
        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        #Publicadores
        self.pub=rospy.Publisher('/duckiebot/camera_node/raw_camera_deteccion_amarillo', Image, queue_size=1)
        self.pub2=rospy.Publisher('/duckiebot/camera_node/raw_camera_punto', Point, queue_size=1)
        self.pubgiro= rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        self.pubpunto=rospy.Publisher('/duckiebot/geometry_msgs/posicionciudadano', Point, queue_size=1)
        self.pubcanny=rospy.Publisher('/duckiebot/camera_node/raw_camera_canny', Image, queue_size=1)
        
        self.pubgray1=rospy.Publisher('/duckiebot/camera_node/raw_camera_imagengray1', Image, queue_size=1)
        self.pubgray=rospy.Publisher('/duckiebot/camera_node/raw_camera_imagengray', Image, queue_size=1)
        self.pubcontornos=rospy.Publisher('/duckiebot/camera_node/raw_camera_imagencontornos', Image, queue_size=1)
        self.min_area=200


    def process_image(self,img):
        #Se cambiar mensage tipo ros a imagen opencv
        self.cv_image = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        #Se deja en frame la imagen actual
        frame = self.cv_image
        self.images.append(frame)
        if len(self.images) == 2:
            self.images = self.images[1:]

    def process_image_count(self,img):

        #Se deja en frame la imagen actual
        frameentero= self.cv_image
        
        #Restringimos el area de la imagen que procesamos
        frame=frameentero[:200,:,:]
        frame2=frame.copy()
        framerojo1=frame[:,:,2]
        frameequa=cv2.equalizeHist(framerojo1)
        frame2[:,:,2]=frameequa


        #Cambiar tipo de color de BGR a HSV
        image=cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        
        # Filtrar colores de la imagen en el rango utilizando 
        mask1 = cv2.inRange(image, lower_red, upper_red)
        mask2 = cv2.inRange(image, lower_redm, upper_redm)
        mask = cv2.bitwise_or(mask1,mask2)
        # Bitwise-AND mask and original image
        

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        kernel2 = np.ones((3,3),np.uint8)
         #Operacion morfologica dilate
        #img_1 = cv2.dilate(mask, kernel, iterations = 3)
        #img_2=cv2.erode(img_1, kernel2, iterations = 2)
        img_out = cv2.dilate(mask, kernel2, iterations = 6)
        
        
       
        segment_image = cv2.bitwise_and(frame,frame, mask=img_out)

        #Ocupamos solo la capa verde del frame
        framerojo=segment_image[:,:,1]
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
        msg_imagenborde=self.bridge.cv2_to_imgmsg(canny_out, "bgr8")
        msg_imagengray=self.bridge.cv2_to_imgmsg(equa, "mono8")
        msg_imagengray1=self.bridge.cv2_to_imgmsg(segment_image, "bgr8")
        self.pubcanny.publish(msg_imagenborde)
        self.pubgray1.publish(msg_imagengray1)
        self.pubgray.publish(msg_imagengray)
        image, contours, hierarchy = cv2.findContours(equa,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        x=0
        y=0
        w=0
        h=0
        for cnt in contours:
                #Obtener rectangulo
                x,y,w,h = cv2.boundingRect(cnt)
                blob=canny_out[x:x+w,y:y+h,:]
                if w>h:
                    resize=w
                    res=cv2.resize(blob,None,fx=2, fy=2, interpolation = cv2.INTER_CUBIC)
        
        dismin=15 #distancia minima entre circulos
        circles = cv2.HoughCircles(equa,cv2.HOUGH_GRADIENT,1,dismin,param1=250,param2=23,minRadius=5,maxRadius=45)

        if not circles == None:
            circles = np.uint16(np.around(circles))
            k=0
            #circles = np.uint16(circles)
            for i in circles[0,:]:
                k+=1
                # draw the outer circle
                cv2.circle(frameentero,(i[0],i[1]),i[2],(0,255,0),2)
                cv2.circle(canny_out,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(frameentero,(i[0],i[1]),2,(0,0,255),3)
            return k
        else:
            return 0
        
        msg_imagencontornos=self.bridge.cv2_to_imgmsg(frameentero, "bgr8")
        self.pubcontornos.publish(msg_imagencontornos)
        self.imagen=msg_imagencontornos
    def process_button(self, joy_msg):
        if joy_msg.buttons[0]:
            local_images = self.images[:]
            cuenta=[]
            for i, image in enumerate(local_images):
                N=self.process_image_count(local_images[i])
                cuenta.append(N)
            cuenta1=np.array(cuenta)
            count=np.median(cuenta1)
            print count
def main():

    rospy.init_node('BlobColor')

    BlobColor()

    rospy.spin()

if __name__ == '__main__':
    main()
