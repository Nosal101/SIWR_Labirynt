#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pgmpy.models import MarkovModel
from pgmpy.factors.discrete import DiscreteFactor
from pgmpy.inference import Mplp
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy import qos
from nav_msgs.msg import Odometry 
# from sensor_msgs.msg import Imu
import gtsam
import os

class PositionPredictor(Node):

    def __init__(self):
        super().__init__('position_predictor')
        if os.path.exists("odom.txt"):
            os.remove("odom.txt")
        if os.path.exists("amcl.txt"):
            os.remove("amcl.txt")
        if os.path.exists("estimate_x.txt"):
            os.remove("estimate_x.txt")
        if os.path.exists("estimate_z.txt"):
            os.remove("estimate_z.txt")

        # Create a Markov model
        self.model = MarkovModel()

        # Initialize map attributes
        self.width = None
        self.height = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_resolution = None

        qos_profile = qos.QoSProfile(depth=10)
        qos_profile.durability = qos.QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Subscribe to odometry and AMCL pose topics
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        # self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Przykładowe pozycje z czujników
        self.sensor1_positions = []
        self.sensor2_positions = []
        self.sensor3_positions = []
        self.odom_prev_x = 0
        self.odom_prev_y = 0
    

    def map_callback(self, data):
        # Process map data
        self.get_logger().info("Map received")
        self.width = data.info.width
        self.height = data.info.height
        self.map_origin_x = data.info.origin.position.x
        self.map_origin_y = data.info.origin.position.y
        self.map_resolution = data.info.resolution

    def odom_callback(self, data):
        # Extract odometry position
        # print("Odometry received")
        odom_position = data.pose.pose.position
        x = odom_position.x
        y = odom_position.y

        x_dist = x - self.odom_prev_x
        y_dist = y - self.odom_prev_y

        self.odom_prev_x = x
        self.odom_prev_y = y
        with open("odom.txt", "a") as file:
            file.write(f"{x}, {y}\n")

        # print(f"Odometry position: {x}, {y}")
        # Add odometry measurement to the Markov model
        if (len(self.sensor1_positions) > len(self.sensor2_positions)):
            self.sensor2_positions.append((x_dist, y_dist))
            self.sensor3_positions.append((x,y))
        self.add_pose_measurement(x, y, 'odom')

    def amcl_pose_callback(self, data):
        # Extract AMCL estimated position
        # print("AMCL pose received")
        amcl_position = data.pose.pose.position
        x = amcl_position.x
        y = amcl_position.y
        # print(f"Odometry position: {x}, {y}")
        # Add AMCL pose measurement to the Markov model
        with open("amcl.txt", "a") as file:
            file.write(f"{x}, {y}\n")
        self.sensor1_positions.append((x, y))
        self.add_pose_measurement(x, y, 'amcl')

    def add_pose_measurement(self, x, y, source):
        # Tworzymy pusty graf czynników
        graph = gtsam.NonlinearFactorGraph()

        # Definiujemy odchylenia standardowe dla szumów pomiarowych
        measurement_noise_sigma = 0.0
        measurement_noise = gtsam.noiseModel.Isotropic.Sigma(2, measurement_noise_sigma)
        initial_estimate = gtsam.Values()

        for i, (pos1, pos2, pos3) in enumerate(zip(self.sensor1_positions, self.sensor2_positions, self.sensor3_positions)):
            key1 = gtsam.symbol('x', i)
            key2 = gtsam.symbol('z', i)
            initial_estimate.insert(key1, np.array(pos1))
            initial_estimate.insert(key2, np.array(pos3))
            
            graph.add(gtsam.PriorFactorPoint2(key1, np.array(pos1), measurement_noise))
            graph.add(gtsam.PriorFactorPoint2(key2, np.array(pos3), measurement_noise))

            if i > 0:
                last_key_x = gtsam.symbol('x', i-1)
                last_key_z = gtsam.symbol('z', i-1)
                graph.add(gtsam.BetweenFactorPoint2(last_key_x, key1, np.array(pos2), measurement_noise))
                # graph.add(gtsam.BetweenFactorPoint2(last_key_z, key2, np.array(pos2), measurement_noise))

            graph.add(gtsam.BetweenFactorPoint2(key1, key2, np.array(pos2), measurement_noise))

        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate)
        result = optimizer.optimize()
        print(result)
        if len(result.keys()) > 0:
            last_key_x = gtsam.symbol('x', int(len(result.keys())/5-1))
            last_key_z = gtsam.symbol('z', int(len(result.keys())/5-1))
            with open("estimate_x.txt", "a") as file:
                file.write(f"{result.atPoint2(last_key_x)[0]}, {result.atPoint2(last_key_x)[1]}\n")
            with open("estimate_z.txt", "a") as file:
                file.write(f"{result.atPoint2(last_key_z)[0]}, {result.atPoint2(last_key_z)[1]}\n")



        # # Dodajemy zmienne (węzły) do grafu czynników
        # initial_estimate = gtsam.Values()
        # for i, (pos1, pos2) in enumerate(zip(self.sensor1_positions, self.sensor2_positions)):
        #     key1 = gtsam.symbol('x', i)
        #     key2 = gtsam.symbol('y', i)
        #     initial_estimate.insert(key1, np.array(pos1))
        #     initial_estimate.insert(key2, np.array(pos2))
        #     # Dodajemy czynniki pomiarowe do grafu
        #     graph.add(gtsam.PriorFactorPoint2(key1, np.array(pos1), measurement_noise))
        #     graph.add(gtsam.PriorFactorPoint2(key2, np.array(pos2), measurement_noise))
        #     #Unaryfactor

        # # Optymalizujemy graf czynników
        # optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate)
        # result = optimizer.optimize()
        # print(result)


        # estimated_positions = []

        # for i in range(int(len(result.keys())/2)):
        #     key1 = gtsam.symbol('x', i)
        #     key2 = gtsam.symbol('y', i)
        #     estimated_pos1 = result.atPoint2(key1)
        #     estimated_pos2 = result.atPoint2(key2)
        #     # Średnia estymowana pozycja
        #     estimated_position = (np.array(estimated_pos1) + np.array(estimated_pos2)) / 2
        #     estimated_positions.append(estimated_position)
        # if len(estimated_positions) > 0:
        #     final_estimated_position = estimated_positions[-1]
        #     print(f"Final estimated position: {final_estimated_position}")

        # # Wyświetlamy wyniki
        # for i in range(len(self.sensor1_positions)):
        #     key1 = gtsam.symbol('x', i)
        #     key2 = gtsam.symbol('y', i)
        #     estimated_pos1 = result.atPoint2(key1)
        #     estimated_pos2 = result
        #     estimated_pos2 = result.atPoint2(key2)

        # print(f"Estimated position for sensor1 node {i}: {estimated_pos1}")
        # print(f"Estimated position for sensor2 node {i}: {estimated_pos2}")



def main(args=None):
    rclpy.init(args=args)
    node = PositionPredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



