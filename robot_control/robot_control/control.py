#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import threading
import time

class s_data(Node):
    def __init__(self):
        super().__init__('s_data')
        self.laser_scan: LaserScan = None
        self.odom: Odometry = None

        self.new_odom = False
        self.new_imu = False

        self.odometry_sub = self.create_subscription(Odometry, "/odom", self.odom_callback,
                                                       qos_profile=qos_profile_sensor_data)
        
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos_profile=HistoryPolicy.KEEP_LAST)
    
    def stop(self):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        
        self.cmd_vel_pub.publish(msg)

    def run_forward(self, vel=0.4):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = vel
        
        self.cmd_vel_pub.publish(msg)

    def turn_left(self, vel = 0.0, ang_vel = 0.2):
        msg = Twist()
        msg.angular.z = ang_vel
        msg.linear.x = vel
        
        self.cmd_vel_pub.publish(msg)

    def turn_right(self, vel = 0.0, ang_vel = .2):
        msg = Twist()
        msg.angular.z = -ang_vel
        msg.linear.x = vel
        
        self.cmd_vel_pub.publish(msg)

    def odom_callback(self, data):
        self.odom = data
        self.new_odom = True
    
    def imu_callback(self, data):
        self.imu = data
        self.new_imu = True


    def process(self):
        if self.new_odom :
            print("Available commands:")
            print("1: Forward")
            print("2: Turn left")
            print("3: Turn right")
            print("4: Stop")
            
            cmd = input("Enter command: ")
            
            if cmd == '1':
                self.run_forward()
            elif cmd == '2':
                self.turn_left()
            elif cmd == '3':
                self.turn_right()
            elif cmd == '4':
                self.stop()
            
            self.new_odom = False
            self.new_imu = False


def main(args=None):
    print("Starting")
    rclpy.init(args=args)

    node = s_data()
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