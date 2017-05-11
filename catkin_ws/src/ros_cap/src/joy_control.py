#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
manzana=0
def main():
    print "hola"
    rospy.init_node('nodo')
    rospy.loginfo('nodo')
    # Subscriber for joint states
    sub = rospy.Subscriber('joy', Joy, process_callback)
    rospy.spin()

def process_callback(msg):
    rospy.loginfo(msg)
    x=msg.axes(1)
    y=msg.axes(0)
    # Base cmd
    base_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
    # Publish each sec
    msj = Twist2DStamped()
    msj.header.stamp = rospy.get_rostime() 
    msj.omega=y*9
    msj.v=x
    rospy.loginfo(msj)
    base_pub.publish(msj)

if __name__ == '__main__':
    main()
