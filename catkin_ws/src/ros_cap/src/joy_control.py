#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
def main():
    print "hola"
    rospy.init_node('joy_control')
    # Subscriber for joint states
    sub = rospy.Subscriber('/duckiebot/joy', Joy, process_callback)
    rospy.spin()

def process_callback(msg):
    rospy.loginfo(msg)
    x=msg.axes[1]
    y=msg.axes[3]
    # Base cmd
    base_pub = rospy.Publisher('/duckiebot/possible_cmd', Twist2DStamped, queue_size=1)
    # Publish each sec
    msj = Twist2DStamped()
    msj.header.stamp = rospy.get_rostime() 
    msj.omega=y*4
    msj.v=x*0.5
    rospy.loginfo(msj)
    base_pub.publish(msj)

if __name__ == '__main__':
    main()
