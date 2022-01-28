#! /usr/bin/env python

import rospy
import time as tm
from std_msgs.msg import Int32 ,String

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/counter', String, queue_size=1)
rate = rospy.Rate(2)
count = Int32()
count.data = 0

while not rospy.is_shutdown(): 
  pub.publish(str(count)+" "+str(tm.time()))
  count.data += 1
  rate.sleep()