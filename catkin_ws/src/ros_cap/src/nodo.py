#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
manzana=0
def main():
    print "hola"
    rospy.init_node('nodo')
    rospy.loginfo('nodo')
    # Subscriber for joint states
    sub = rospy.Subscriber('/duckiebot/joy', Joy, process_callback)
    rospy.spin()

def process_callback(msg):
    rospy.loginfo(msg)
    rospy.loginfo(msg.axes)
    x=msg.buttons[0]
    y=msg.axes[3]
    # Base cmd
    base_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
    # Publish each sec
    msj = Twist2DStamped()
    msj.header.stamp = rospy.get_rostime()     
    msj.omega = 10*y
    global manzana
    if x==1:
        manzana=manzana+0.1
    else:
        manzana=0
    msj.v=manzana
    rospy.loginfo(msj)
    base_pub.publish(msj)

if __name__ == '__main__':
    main()
