#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy


def main():
	rospy.init_node('test_1')
	rospy.loginfo('test_1')
    # Subscriber for joint states
	sub = rospy.Subscriber('/duckiebot/joy', Joy, process_callback)
	rospy.spin()
    # Base cmd
    
    # Publish each sec
    

def process_callback(msg):
	base_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
	RT = msg.axes[5]
	LT = msg.axes[2]
	joystickomega= msg.axes[0]
	msg1 = Twist2DStamped()
	msg1.header.stamp = rospy.get_rostime()
	msg1.omega = joystickomega
	#msg1.omega = 0.9
	msg1.v = (-RT)+(LT)
	base_pub.publish(msg1)
    

if __name__ == '__main__':
    main()
