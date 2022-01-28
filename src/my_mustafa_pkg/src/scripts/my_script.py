#!/usr/bin/env python
import rospy
import time
import math
import numpy as np                                        
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32 ,String
from geometry_msgs.msg import Twist


class MoveTurtlebot():

      
    def __init__(self):
    
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_subs = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.callback)
        self.last_cmdvel_command = Twist()
        self.cmd = Twist()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10)
        self._check_laser_ready()
        #self.laser_msg = rospy.wait_for_message("scan", LaserScan, timeout=5.0)


    def _check_laser_ready(self):
        self.laser_msg = None
        rospy.loginfo("Checking Laser...")
        while self.laser_msg is None and not rospy.is_shutdown():
            try:
                self.laser_msg = rospy.wait_for_message("/kobuki/laser/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /kobuki/laser/scan READY=>" + str(self.laser_msg))

            except:
                rospy.logerr("Current /kobuki/laser/scan not ready yet, retrying for getting scan")
        rospy.loginfo("Checking Laser...DONE")
        return self.laser_msg


    def avoid_collision(self,collision_direct="Right",obstacle_distance=1,speed=1):
        while True:
            forward=self.get_front_laser()
            left_right_read=self.get_left_right_readings()
            print("forward: ",forward," left right read : ",left_right_read)
            if forward>obstacle_distance and math.isinf(left_right_read[1]) and math.isinf(left_right_read[0]) :
                self.move_straight(speed)

            elif forward <= obstacle_distance:
                self.stop_turtlebot()
                self.turn_z(collision_direct,1.0)
            elif forward > obstacle_distance and ( left_right_read[1] > 0.4 and left_right_read[0] > 0.4):
                
                self.move_straight(speed)

            else:
                self.stop_turtlebot() 

    def callback(self,msg):
        self.laser_msg = msg
        # print(msg.ranges[360])                                  # Define a function called 'callback' that receives a parameter 

    def get_laser(self):
        time.sleep(1)
        return self.laser_msg.ranges

    def get_laser_pose(self,pose):
        time.sleep(1)
        return self.laser_msg.ranges

    def get_front_laser(self):
        list_arr=list(self.laser_msg.ranges)                                       # named 'msg'
        narr=np.array(list_arr)
        wall_dist=narr[350:370].mean()
        self.front_distance=wall_dist
        # print (list_arr[205:440])                                  # Print the value 'data' inside the 'msg' parameter
        # print (list_arr[350:370])  
        return self.front_distance                               # Print the value 'data' inside the 'msg' parameter
      

    def get_left_right_readings(self):
        
        
        left_read=list(self.laser_msg.ranges[715:])
        right_read=list(self.laser_msg.ranges[:5])
        # print(np.mean(left_read))
        # print(np.mean(right_read))
        return (np.mean(left_read),np.mean(right_read))
        
    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.scan_subs.get_num_connections()
            print("publih connections "+str(connections))
            if connections > 0:
                self.cmd_vel_pub.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                # print(self.get_front_laser())
                break
            else:
                # print(self.get_front_laser())
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shut_down()
        self.stop_turtlebot()
        self.ctrl_c = True

    def stop_turtlebot(self):
        rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_x_time(self, moving_time, linear_speed=0.2, angular_speed=0.2):

        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed

        self.publish_once_in_cmd_vel()

        #self.subs_once_in_laserscan()
        time.sleep(moving_time)
    def move_straight(self,speed):

        # Initilize velocities
        self.cmd.linear.x = speed
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()
    
    def turn_z(self, direction, speed):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        if direction == "Left":
            self.cmd.angular.z = -speed
            
        else:
            self.cmd.angular.z = speed

        
        
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while not (math.isinf(self.get_front_laser())):

            # Publish the velocity
            self.cmd_vel_pub.publish(self.cmd)
        
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_turtlebot()

        s = "Turned robot " + str(direction) 
        print(s)
        return s
    def move_square(self, square_size=0):

        i = 0

        while not self.ctrl_c and i < 4:
            # Move Forward
            self.move_x_time(moving_time=square_size, linear_speed=0.2, angular_speed=0.0)
            # Turn, the turning is not affected by the length of the side we want
            self.move_x_time(moving_time=10.0, linear_speed=0.0, angular_speed=-0.157)#turn -90 degree left
            i += 1

        self.stop_turtlebot()
        rospy.loginfo("######## Finished Moving in a Square")

if __name__ == '__main__':
    rospy.init_node('move_turtlebot', anonymous=True)
    moveturtlebot_object = MoveTurtlebot()
    try:
        # moveturtlebot_object.move_square(5)
    #    print( moveturtlebot_object.get_left_right_readings())
        moveturtlebot_object.avoid_collision()
    except rospy.ROSInterruptException as e:
        print(e)



