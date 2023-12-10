#!/usr/bin/env python3

import rospy, math, random
import numpy as np

from std_msgs.msg import Int16

from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3

from sensor_msgs.msg import LaserScan

import time
import sys

from enum import Enum
class PathState(Enum):
    FWD = 0
    REV = 1

class CmdBot():
    def __init__(self):
        rospy.init_node('bot_commander')
        
        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.prev_time = rospy.Time.now()
        self.last_start_time = rospy.Time.now()
        self.state = PathState.FWD
        
        rospy.loginfo('Initializing bot commander.')
        
        self.bot_rad = 0
        self.botx = 0
        self.boty = 0    
                
    def odom_callback(self, data):
        self.botx = data.pose.pose.position.x
        self.boty = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion (orientation_list)
        self.bot_rad = yaw

    
    def update_cmd(self):
        
        t2 = rospy.Time.now()
        t1 = self.prev_time
        self.prev_time = t2
        dt = (t2-t1).to_sec()
        
        state_duration = (t2 - self.last_start_time).to_sec()
        
        cmd = Twist()
        
        if self.state is PathState.FWD:
            cmd.linear.x = 1.0;
            cmd.angular.z = 1.0; # rad/sec
            if state_duration > 3.0:
                self.state = PathState.REV
                self.last_start_time = t2
        else:
            cmd.linear.x = -1.0;
            cmd.angular.z = -1.0; # rad/sec
            if state_duration > 3.0:
                self.state = PathState.FWD
                self.last_start_time = t2

        self.cmd_pub.publish(cmd)
        

if __name__ == '__main__':
    try:
        cmd_bot = CmdBot()
        rospy.loginfo("Starting bot commander")

        r = rospy.Rate(10.0) #Task rate in Hz
        while not rospy.is_shutdown():
            cmd_bot.update_cmd()
            r.sleep()
            
    except rospy.ROSInterruptException:
        rospy.logerr("CmdBot failed to start")
        pass
