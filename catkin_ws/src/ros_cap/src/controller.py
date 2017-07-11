#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
class controller():

    def __init__(self):
        self.detector_sub=rospy.Subscriber('distancia3d', Point, self.image_callback)
        self.joy_sub=rospy.Subscriber('/duckiebot/possible_cmd', Twist2DStamped, self.joy_callback)
        self.final_pub=rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        self.duck_detect=False
        self.v=0
        self.omega=0
    
    def image_callback(self, msg):
        rospy.loginfo(msg)
        if msg.z<=20:
            self.duck_detect=True
        else:
            self.duck_detect=False
    
    def joy_callback(self, msg):
        self.omega=msg.omega
        if msg.v>0 and self.duck_detect==True:
            self.omega=-1
        else:
            self.v=msg.v
        msj= Twist2DStamped()
        msj.v=self.v
        msj.omega=self.omega
        self.final_pub.publish(msj)

def main():

    rospy.init_node('controller')

    controller()

    rospy.spin()

if __name__ == '__main__':
    main()
