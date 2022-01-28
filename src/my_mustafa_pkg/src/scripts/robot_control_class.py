#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import numpy as np


class RobotControl():

    def __init__(self, robot_name="turtlebot"):
        rospy.init_node('robot_control_node', anonymous=True)

        if robot_name == "summit":
            rospy.loginfo("Robot Summit...")
            cmd_vel_topic = "/summit_xl_control/cmd_vel"
            # We check sensors working
            self._check_summit_laser_ready()
        else:      
            rospy.loginfo("Robot Turtlebot...")      
            cmd_vel_topic='/cmd_vel'
            # We check sensors working
            self._check_laser_ready()

        # We start the publisher
        self.vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.cmd = Twist()        

        self.laser_subscriber = rospy.Subscriber(
            '/kobuki/laser/scan', LaserScan, self.laser_callback)
        self.summit_laser_subscriber = rospy.Subscriber(
            '/hokuyo_base/scan', LaserScan, self.summit_laser_callback)
        
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        self.sleeptime=time.sleep(1.0) # for 1 seconds
        rospy.on_shutdown(self.shutdownhook)

    
    def _check_summit_laser_ready(self):
        self.summit_laser_msg = None
        rospy.loginfo("Checking Summit Laser...")
        while self.summit_laser_msg is None and not rospy.is_shutdown():
            try:
                self.summit_laser_msg = rospy.wait_for_message("/hokuyo_base/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /hokuyo_base/scan READY=>" + str(self.summit_laser_msg))

            except:
                rospy.logerr("Current /hokuyo_base/scan not ready yet, retrying for getting scan")
        rospy.loginfo("Checking Summit Laser...DONE")
        return self.summit_laser_msg

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

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shut_down()
        self.stop_turtlebot()
        self.ctrl_c = True

    def laser_callback(self, msg):
        self.laser_msg = msg

    def summit_laser_callback(self, msg):
        self.summit_laser_msg = msg

    def get_laser(self, pos):
        time.sleep(1)
        return self.laser_msg.ranges[pos]

    def get_laser_summit(self, pos):
        time.sleep(1)
        return self.summit_laser_msg.ranges[pos]

    def get_front_laser(self):
        time.sleep(1)
        return self.laser_msg.ranges[360]

    def get_laser_full(self):
        time.sleep(1)
        return self.laser_msg.ranges

    def radian_rotateAngle(self):
        pass
        
    def stop_robot(self):
        rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = 0.0
        self.cmd.angular.x = 0.0
        self.cmd.angular.y = 0.0
        self.cmd.angular.z = 0.0
        
        self.publish_once_in_cmd_vel()

    def move_straight(self):

        # Initilize velocities
        self.cmd.linear.x = 0.5
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()
    def move_straight_(self,speed):

        # Initilize velocities
        self.cmd.linear.x = speed
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()

    def move_straight_y(self,speed):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = speed
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()
    
   
    def move_straight_time(self, motion, speed, time):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "forward" or motion==1:
            self.cmd.linear.x = speed
        elif motion == "backward" or motion==2:
            self.cmd.linear.x = - speed
        elif motion == "right" or motion==3:
            self.cmd.linear.y = speed
        elif motion == "left" or motion==4:
            self.cmd.linear.y = - speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + str(motion) + " for " + str(time) + " seconds"
        print(s)
        return s


    def select_move(self,select, cordinate,motion,clockwisez, speed, time):
        

        if select=="linear" or select==1:

            if motion == "forward" or motion==1:
               self.move_straight_time(motion,speed,time)
            elif motion == "backward" or motion==2:
               self.move_straight_time(motion,-speed,time)
            elif motion == "right" or motion==3:
               self.move_straight_time(motion,-speed,time)
            elif motion == "left" or motion==4:
               self.move_straight_time(motion,speed,time)
               
            else:
               self.stop_robot()

            s = "Moved linear robot " + str(motion) + " for " + str(time) + " seconds"+"by velocity " +str(speed)

        else:

            if cordinate == "x":
               self.turn_x(clockwisez,speed,time)
            elif cordinate == "y":
                self.turn_y(clockwisez,speed,time)
            elif cordinate == "z":
                self.turn_z(clockwisez,speed,time)
            
            else :
                self.stop_robot()
            
            s = "Moved angular robot " + cordinate + " for " + str(time) + " seconds"+"by velocity " +str(speed)+ " clockwise true " +str(clockwisez)


        
        return s


    def turn_z(self, clockwise, speed, time):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        if clockwise == True:
            self.cmd.angular.z = -speed
            
        else:
            self.cmd.angular.z = speed

        
        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Turned robot " + str(clockwise) + " for " + str(time) + " seconds"
        print(s)
        return s

    def turn_z_(self, clockwise, speed):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        if clockwise == True:
            self.cmd.angular.z = speed
            
        else:
            self.cmd.angular.z = -speed

        

        # Publish the velocity
        self.vel_publisher.publish(self.cmd)
            

        s = "Turned robot " + str(clockwise) + " for " + str(speed) + "  velocity"
        print(s)
        
    def turn_y(self, clockwise, speed, time):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.z = 0

        if clockwise == True:
            self.cmd.angular.y = -speed
            
        else:
            self.cmd.angular.y = speed

        
        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.sleeptime

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Turned robot " + str(clockwise) + " for " + str(time) + " seconds"
        print(s)
        return s

    def turn_x(self, clockwise, speed, time):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if clockwise == True:
            self.cmd.angular.x = -speed
            
        else:
            self.cmd.angular.x = speed

        
        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.sleeptime

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Turned robot " + str(clockwise) + " for " + str(time) + " seconds"
        print(s)
        return s
        
    def get_laser_readings(self):
        
        iterable_right=[i for i in range(0,10)]
        iterable_left=[i for i in range(710,720)]
        left_read=list(map(lambda pose: self.get_laser(pose),iterable_left))
        right_read=list(map(lambda pose: self.get_laser(pose),iterable_right))

        return (np.mean(left_read),np.mean(right_read))
    
    
    def inf_move_straight(self):

        while True:
           
            (left,right)=self.get_laser_readings()
            if right==float("inf") and left==float("inf"):
                break
            self.move_straight()
        self.stop_robot()

    def get_highest_lowest(self):

        listarray=list(self.get_laser_full())
        # initialize tmporary variable 
        temp=0
        min_v=listarray[0]
        min_i=0
        max_v=listarray[0]
        max_i=0
        for (i,item) in enumerate(listarray):
            #print(str(i)+" "+str(item)+"\n")
            if item < min_v:
                min_v=item
                min_i=i
            if not math.isinf(item) and item > max_v:
                max_v=item
                max_i=i
        result=(max_i,min_i)
        return result

    def avoid_collision(self,obstacle_distance=1,clockwise=False):
        while True:
            forward=self.get_front_laser()

            if forward<obstacle_distance:
                break
            self.move_straight()
            self.rate.sleep()

        self.stop_robot()
        self.turn_z(clockwise,0.177,10)

if __name__ == '__main__':
    print("python dogrudan bu dosyadan(robot_class file) çalıştırıldı")
    print(__name__);
    rbt=RobotControl(robot_name="turtlebot")
    try:
        rbt.avoid_collision(1.0,True)
    except rospy.ROSInterruptException as err:
        print(err.__class__)
        print(err)
else:
    print("python (robot_clas) modülü import edilerek çalıştırıldı")
    print(__name__);




