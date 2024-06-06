#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import threading
import time

class Traverse(Node):
    def __init__(self):
        super().__init__('traverse')
        self.laser_scan: LaserScan = None
        self.odom: Odometry = None

        self.new_scan = False
        self.new_odom = False

        self.laser_scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback,
                                                       qos_profile=qos_profile_sensor_data)
        
        self.odometry_sub = self.create_subscription(Odometry, "/odom", self.odom_callback,
                                                       qos_profile=qos_profile_sensor_data)
        
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos_profile=HistoryPolicy.KEEP_LAST)

        self.turn_right(vel = 0.0)
        time.sleep(3.14)
        self.stop()
        
    
    def stop(self):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        
        self.cmd_vel_pub.publish(msg)

    def run_forward(self, vel=0.1):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = vel
        
        self.cmd_vel_pub.publish(msg)

    def turn_left(self, vel = 0.1, ang_vel = 0.5):
        msg = Twist()
        msg.angular.z = ang_vel
        msg.linear.x = vel
        
        self.cmd_vel_pub.publish(msg)

    def turn_right(self, vel = 0.1, ang_vel = .5):
        msg = Twist()
        msg.angular.z = -ang_vel
        msg.linear.x = vel
        
        self.cmd_vel_pub.publish(msg)

    def scan_callback(self, data):
        self.laser_scan = data
        self.new_scan = True

    def odom_callback(self, data):
        self.odom = data
        self.new_odom = True

    def get_index(self,angle):
        angle_min = self.laser_scan.angle_min
        angle_max = self.laser_scan.angle_max
        return ((angle_max - angle_min) / self.laser_scan.angle_increment)*(angle/360)

    def process(self):
        if self.new_scan and self.new_odom:
            print('jestem w pentli')
            # self.new_scan = False
            # self.new_odom = False

            # r1 = self.laser_scan.ranges[int(self.get_index(90))]
            # r2 = self.laser_scan.ranges[int(self.get_index(60))]
            # l1 = self.laser_scan.ranges[int(self.get_index(270))]
            # l2 = self.laser_scan.ranges[int(self.get_index(300))]

            # d1 = (l1/r1) - 1
            # d2 = (l2/r2) - 1

            # a1 = 0.5
            # a2 = 1.0
            # r = a1*d1 + a2*d2

            # precision=0.01

            # if r>precision:
            #     self.turn_right(vel=0.8)  
            # elif r<-precision:
            #     self.turn_left(vel=0.8)

            #print(self.laser_scan.ranges[0])
            #print(self.laser_scan.range_max)
            #angle_min = self.laser_scan.angle_min
            #angle_max = self.laser_scan.angle_max
            # angle_inc = self.laser_scan.angle_increment
            # ranges = self.laser_scan.ranges

            # position = self.odom.pose.pose.position
            # self.run_forward()



def main(args=None):
    rclpy.init(args=args)

    node = Traverse()
    rate = node.create_rate(100)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    while rclpy.ok():
        node.process()
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()