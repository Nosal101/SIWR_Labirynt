#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import threading
import time
import numpy as np
import matplotlib.pyplot as plt

class Maps(Node):
    def __init__(self):
        super().__init__('Maps')
        self.laser_scan: LaserScan = None
        self.odom: Odometry = None
        self.imu: Imu = None

        self.scans = []
        self.robot_position = (0, 0) #Startowa pozycja robota

        self.new_scan = False
        self.new_odom = False
        self.new_imu = False

        self.laser_scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback,
                                                       qos_profile=qos_profile_sensor_data)
        
        self.odometry_sub = self.create_subscription(Odometry, "/odom", self.odom_callback,
                                                       qos_profile=qos_profile_sensor_data)
        
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback,
                                                       qos_profile=qos_profile_sensor_data)
        

    def scan_callback(self, data):
        self.laser_scan = data
        self.new_scan = True

    def odom_callback(self, data):
        self.odom = data
        self.new_odom = True

    def imu_callback(self, data):
        self.imu = data
        self.new_imu = True

    def save_map_to_file(self, filename='map_points.csv'):
        # Konwersja listy punktów na jedną tablicę numpy
        all_points = np.array([point for scan in self.scans for point in zip(*scan)])
        
        # Zapis do pliku CSV
        np.savetxt(filename, all_points, delimiter=',', header='x,y', comments='')


    def process(self):
        if self.new_scan and self.new_odom and self.new_imu:
            # Konwersja skanu z lidaru do współrzędnych XY
            angles = np.linspace(self.laser_scan.angle_min, self.laser_scan.angle_max, len(self.laser_scan.ranges))
            ranges = np.array(self.laser_scan.ranges)
            robot_orientation = self.odom.pose.pose.orientation.z  # Uwzględnienie orientacji robota

            x = ranges * np.cos(angles + robot_orientation)
            y = ranges * np.sin(angles + robot_orientation)

            # Uwzględnienie przemieszczenia robota w obliczeniach pozycji punktów
            robot_x = self.odom.pose.pose.position.x
            robot_y = self.odom.pose.pose.position.y
            

            # Zaktualizuj pozycję robota
            self.robot_position = (robot_x, robot_y)

            # Dodanie pozycji robota do danych
            x += robot_x
            y += robot_y

            # Dodanie nowych punktów do listy skanów
            self.scans.append((x, y))

            self.plot_map()

            self.new_scan = False
            self.new_odom = False
            self.new_imu = False

    def plot_map(self):
        plt.figure(figsize=(10, 10))
        for scan in self.scans:
            x, y = scan
            plt.scatter(x, y, c='blue', s=5)
        plt.scatter(*self.robot_position, c='red', s=50, label='Robot')
        plt.title('Mapa otoczenia')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.grid(True)
        plt.legend()
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

def main(args=None):
    rclpy.init(args=args)

    node = Maps()
    rate = node.create_rate(100)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    try:
        while rclpy.ok():
            node.process()
            rate.sleep()
    finally:
        node.save_map_to_file('map_points.csv')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
