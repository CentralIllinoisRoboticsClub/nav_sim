import rclpy
from rclpy.node import Node
import rclpy.time
import math, random
import numpy as np

from std_msgs.msg import Int16

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3

from sensor_msgs.msg import LaserScan

# https://github.com/ros/geometry2/issues/222
# https://github.com/davheld/tf/blob/master/src/tf/transformations.py
import transformations # for euler to quat

from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import time
import sys

# For py and c++ in one package
# https://github.com/tanyouliang95/MySandBox/tree/master/ros2_payload

# https://github.com/ros2/geometry2/tree/ros2/examples_tf2_py/examples_tf2_py
# https://github.com/ros2/geometry2/blob/ros2/examples_tf2_py/examples_tf2_py/dynamic_broadcaster.py

class SimBot(Node):
    def __init__(self):
        super().__init__('bot_simulator')
        
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.sim_cmd_callback, 1)
        self.cmd_sub # prevent unused var warning
        
        self.USE_SIMPLE_SCAN_SIM = False
        
        if(self.USE_SIMPLE_SCAN_SIM):
            self.scan_pub = self.create_publisher(LaserScan, 'noise_scan', 50)
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 5)
        self.odom_broadcaster = TransformBroadcaster(self)
        self.tfs = TransformStamped()
        self.tfs.header.frame_id = "odom"
        self.tfs.child_frame_id = "base_link"
        self.tfs.transform.translation.z = 0.0
        
        now_stamp = self.get_clock().now().to_msg()
        self.prev_time = now_stamp
        self.scan_time1 = now_stamp
        
        self.get_logger().info('Initializing bot simulator.')
        
        self.v = 0
        self.w = 0
        self.v_cmd = 0
        self.w_cmd = 0
        
        self.max_accel= 2.
        self.max_w_dot = 2.
        
        self.bot_rad = 0
        self.botx = 0
        self.boty = 0    
        
        self.noise_scan_active = False
        self.active_duration = 0.0
        self.check_time = now_stamp
        self.noise_scan_start_time = now_stamp
        
        self.timer = self.create_timer(1./50., self.update_odom)
    
    def dt_to_sec(self, stampA, stampB):
        return stampA.sec + stampA.nanosec * 10**-9 - stampB.sec - stampB.nanosec * 10**-9
                
    def sim_cmd_callback(self, data):
        v = data.linear.x
        w = data.angular.z
        
        # Turnig Radius limit = 0.5 meters
        MAX_CURVATURE = 2.0
        #~ if(abs(v) < 0.1 and abs(w) > 0.0):
            #~ v = 0.5*np.sign(v)
            #~ if(v == 0.0):
                #~ v = 0.5
        #~ elif(abs(v) > 0):
            #~ curv = w/v
            #~ if(abs(v) < 0.5):
                #~ v = np.sign(v)*0.5
            #~ if(abs(curv) > MAX_CURVATURE):
                #~ w = np.sign(curv)*v*MAX_CURVATURE
        if(abs(v) > 0 and abs(v) < 0.3):
            curv = w/v
            if(abs(curv) > MAX_CURVATURE):
                curv = np.sign(curv)*MAX_CURVATURE
            v = np.sign(v)*0.3
            #w = curv*v
            
        self.v_cmd = v
        self.w_cmd = w
    
    def update_odom(self):
        
        t2 = self.get_clock().now().to_msg()
        t1 = self.prev_time
        self.prev_time = t2
        dt = self.dt_to_sec(t2,t1)
        
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
        
        odom_quat = transformations.quaternion_from_euler(0, 0, self.bot_rad)
        
        self.tfs.header.stamp = t2
        self.tfs.transform.translation.x = self.botx
        self.tfs.transform.translation.y = self.boty
        #self.tfs.transform.rotation = odom_quat
        self.tfs.transform.rotation.x = odom_quat[0]
        self.tfs.transform.rotation.y = odom_quat[1]
        self.tfs.transform.rotation.z = odom_quat[2]
        self.tfs.transform.rotation.w = odom_quat[3]
        self.odom_broadcaster.sendTransform(self.tfs)
        
        odom = Odometry()
        odom.header.stamp = t2
        odom.header.frame_id = "odom"

        # set the position
        #odom.pose.pose = Pose( Point(self.botx, self.boty, 0.), Quaternion(self.tfs.transform.rotation) )
        odom.pose.pose.position.x = self.botx
        odom.pose.pose.position.y = self.boty
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = self.tfs.transform.rotation.x
        odom.pose.pose.orientation.y = self.tfs.transform.rotation.y
        odom.pose.pose.orientation.z = self.tfs.transform.rotation.z
        odom.pose.pose.orientation.w = self.tfs.transform.rotation.w

        # set the velocity
        odom.child_frame_id = "base_link"
        #odom.twist.twist = Twist(Vector3(self.v, 0, 0), Vector3(0, 0, self.w))
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        # publish the message
        self.odom_pub.publish(odom)
        
        # LaserScan simulation
        # initially just check point obstacle at 5,0 in odom
        if(self.USE_SIMPLE_SCAN_SIM):
            #if( (t2 - self.scan_time1).to_sec() > 30.0):
            #    self.USE_SIMPLE_SCAN_SIM = False
            scan = LaserScan()
            scan.header.frame_id = 'noise_laser'
            scan.range_min = 0.2
            scan.range_max = 50.0
            fov = 24
            nDet = 8
            
            current_time = self.get_clock().now().to_msg()
            delta_check_time = self.dt_to_sec(current_time, self.check_time)
            delta_active_time = self.dt_to_sec(current_time, self.noise_scan_start_time)
            scan.header.stamp = current_time
            scan.angle_min = -fov/2*3.14/180.0
            scan.angle_max = fov/2*3.14/180.0
            scan.angle_increment = (fov/nDet)*3.14/180.0
            scan.time_increment = 0.001
            scan.ranges = []
            
            # random times we see noise obstacle at 5,0 (if in fov)
            #   The ground arc is also added at the same random time for now.
            #   This state is latched for 1-2 sec
            if(not self.noise_scan_active and delta_check_time > 1.0): #only check the random number every second
                rn  = random.randint(0,10)
                self.get_logger().info('checking random number: %d', rn)
                if(rn > 6):
                    self.noise_scan_active = True
                    self.noise_scan_start_time = current_time
                    self.active_duration = random.random()*2.0 + 1.0
                    self.get_logger().info('active duration: %0.1f',self.active_duration)
                    
                #end if
                self.check_time = current_time
            else:
                if( delta_active_time > self.active_duration ):
                    self.noise_scan_active = False
                #end if
            #end if
            
            if(self.noise_scan_active):
                obstacle_found = False
                dx = 5.0 - self.botx
                if( self.dt_to_sec(t2, self.scan_time1) > 15.0):
                    dx = 2.0 - self.botx
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
                                scan.ranges.append(6.0)
                if not obstacle_found:
                    for k in range(nDet):
                        scan.ranges.append(6.0)
            else:
                for k in range(nDet):
                    scan.ranges.append(0.0)
            #end if active or not
            self.scan_pub.publish(scan)
            
            br = TransformBroadcaster(self)
            roll = 0
            pitch = 0.08
            laser_quat = tf.transformations.quaternion_from_euler(roll, pitch, 0)
            br.sendTransform((0,0,0.3),laser_quat,t2,"noise_laser","base_link")
        

def main(args=None):
    rclpy.init(args=args)
    sim_bot = SimBot()
    sim_bot.get_logger().info("Starting bot simulator")

    rclpy.spin(sim_bot)
    
    sim_bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
