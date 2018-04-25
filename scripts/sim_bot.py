#!/usr/bin/env python

import rospy, math
import numpy as np

from std_msgs.msg import Int16

from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3

from sensor_msgs.msg import LaserScan

import time
import sys

class SimBot():
    def __init__(self):
        rospy.init_node('bot_simulator')
        
        rospy.Subscriber('cmd_vel', Twist, self.sim_cmd_callback, queue_size=1)
        
        self.USE_SIMPLE_SCAN_SIM = False
        if(self.USE_SIMPLE_SCAN_SIM):
            self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
        
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.prev_time = rospy.Time.now()
        
        rospy.loginfo('Initializing bot simulator.')
        
        self.v = 0
        self.w = 0
        self.v_cmd = 0
        self.w_cmd = 0
        
        self.max_accel= 5
        self.max_w_dot = 3
        
        self.bot_rad = 0
        self.botx = 0
        self.boty = 0    
                
    def sim_cmd_callback(self, data):
        v = data.linear.x
        w = data.angular.z
        
        # Turnig Radius limit = 0.5 meters
        MAX_CURVATURE = 2.0
        if(abs(v) < 0.1 and abs(w) > 0.0):
            v = 0.5*np.sign(v)
            if(v == 0.0):
                v = 0.5
        elif(abs(v) > 0):
            curv = w/v
            if(abs(curv) > MAX_CURVATURE):
                w = np.sign(curv)*v*MAX_CURVATURE
            
        self.v_cmd = v
        self.w_cmd = w
    
    def update_odom(self):
        
        t2 = rospy.Time.now()
        t1 = self.prev_time
        self.prev_time = t2
        dt = (t2-t1).to_sec()
        
        accel = (self.v_cmd - self.v)/dt
        mu_v = np.sign(accel)
        if(abs(accel) > self.max_accel):
            accel = self.max_accel*mu_v
        
        dmeters = self.v*dt + 0.5*accel*dt**2
        self.v += accel*dt
        
        w_dot = (self.w_cmd - self.w)/dt
        mu_w = np.sign(w_dot)
        if(abs(w_dot) > self.max_w_dot):
            w_dot = self.max_w_dot*mu_w
        
        dtheta = self.w*dt + 0.5*w_dot*dt**2
        self.w += w_dot*dt

        #update bot position
        self.bot_rad = self.bot_rad + dtheta
        dx = dmeters*np.cos(self.bot_rad)
        dy = dmeters*np.sin(self.bot_rad)
        self.botx = self.botx + dx
        self.boty = self.boty + dy
        
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.bot_rad)
        self.odom_broadcaster.sendTransform(
        (self.botx, self.boty, 0.),
        odom_quat,
        t2,
        "base_link",
        "odom"
        )
        
        odom = Odometry()
        odom.header.stamp = t2
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.botx, self.boty, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.v, 0, 0), Vector3(0, 0, self.w))

        # publish the message
        self.odom_pub.publish(odom)
        
        # LaserScan simulation
        # initially just check point obstacle at 10,0 in odom
        if(self.USE_SIMPLE_SCAN_SIM):
            scan = LaserScan()
            scan.header.frame_id = 'laser'
            scan.range_min = 0.2
            scan.range_max = 30.0
            fov = 48
            nDet = 16
            
            current_time = rospy.Time.now()
            scan.header.stamp = current_time
            scan.angle_min = -fov/2*3.14/180.0
            scan.angle_max = fov/2*3.14/180.0
            scan.angle_increment = (fov/nDet)*3.14/180.0
            scan.time_increment = 0.001
            scan.ranges = []
            
            obstacle_found = False
            dx = 10.0 - self.botx
            dy = 0.0 - self.boty
            theta = math.atan2(dy,dx)
            if(abs(self.bot_rad - theta) < fov/2*3.14/180.0):
                dist = math.sqrt(dx**2+dy**2)
                if(dist < 30.0):
                    obstacle_found = True
                    for sub_theta_deg in range(-fov/2,fov/2+1,fov/nDet):
                        if(abs(self.bot_rad+sub_theta_deg*3.14/180.0 - theta) < scan.angle_increment):
                            scan.ranges.append(dist)
                        else:
                            scan.ranges.append(50.0)
            if not obstacle_found:
                for k in range(nDet):
                    scan.ranges.append(50.0)
            self.scan_pub.publish(scan)
        

if __name__ == '__main__':
    try:
        sim_bot = SimBot()
        rospy.loginfo("Starting bot simulator")

        r = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            sim_bot.update_odom()
            r.sleep()
            
    except rospy.ROSInterruptException:
        rospy.logerr("SimBot failed to start")
        pass
