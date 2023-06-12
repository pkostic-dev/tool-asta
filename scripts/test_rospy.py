#!/usr/bin/env python3

import rospy
import logging
import math
from std_msgs.msg import String
import time


pub = rospy.Publisher('test_topic', String, queue_size=1)

rospy.init_node('test_node')

r = rospy.Rate(100)
while not rospy.is_shutdown():
    t = time.asctime(time.localtime(time.time()))
    pub.publish("msg 1 %s" % t)
    pub.publish("msg 2 %s" % t)
    r.sleep()