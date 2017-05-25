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
    sub = rospy.Subscriber('distancia3d', Point, process_callback)
    rospy.spin()

def process_callback(msg):
    rospy.loginfo(msg)
    x=msg.x
    y=msg.y
    # Base cmd
    base_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
    # Publish each sec
    msj = Twist2DStamped()
    msj.header.stamp = rospy.get_rostime() 
    if not(x==320):
        if x<320:    
            msj.omega = -10*(x-320)/320
        elif x>320:
            msj.omega = -10*(x-320)/320
        
    rospy.loginfo(msj)
    base_pub.publish(msj)

if __name__ == '__main__':
    main()
