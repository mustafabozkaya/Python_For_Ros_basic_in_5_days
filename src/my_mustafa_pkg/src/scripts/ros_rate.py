#!/usr/bin/env python
import rospy
if __name__ == '__main__':
    rospy.init_node("rospy_rate")
    rate = rospy.Rate(1) # ROS Rate at 5Hz
    
    while not rospy.is_shutdown():
        rospy.loginfo("Hello")
        rate.sleep()