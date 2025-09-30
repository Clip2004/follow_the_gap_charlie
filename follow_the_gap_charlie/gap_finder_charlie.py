#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import numpy as np

class GapFinderCharlie(Node): 
    def __init__(self):
        super().__init__("gap_finder_charlie_node")
        self.scan_sub = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.cmd_pos_ctrl_pub = self.create_publisher(Twist, '/cmd_pos_ctrl', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.r = 15
        self.gap_threshold = 1.0
        self.n = 15
        self.max_distance_index = 0
        self.x = 0.0
        self.angle = 0.0

    def get_Gaps(self, msg):
        indexes = []

        # Convertir a numpy array
        ranges_array = np.array(msg.ranges)

        # SOLO trabajar de 0° a 180° → primera mitad del LiDAR
        half_len = len(ranges_array) // 2
        ranges_array = ranges_array[:half_len]

        # Encontrar el índice del valor mínimo
        laser_min_index = np.argmin(ranges_array)

        # Crear copia para no modificar original
        ranges_copy = ranges_array.copy()

        # Aplicar slicing
        start_idx = max(0, laser_min_index - self.r)
        end_idx = min(len(ranges_copy), laser_min_index + self.r + 1)
        ranges_copy[start_idx:end_idx] = 0.0

        consecutive_count = 0
        temp_indexes = []

        for i in range(len(ranges_copy)):
            if ranges_copy[i] > self.gap_threshold:
                consecutive_count += 1
                temp_indexes.append(i)
                if consecutive_count >= self.n:
                    if indexes and indexes[-1] != 0:
                        indexes.append(0)
                    indexes.extend(temp_indexes)
            else:
                consecutive_count = 0
                temp_indexes = []

        return indexes

    def scan_callback(self, msg):
        indexes = self.get_Gaps(msg)
        if not indexes:
            self.get_logger().info("No gaps found")
            return

        max_gap = []
        current_gap = []
        for idx in indexes:
            if idx != 0:
                current_gap.append(idx)
            else:
                if len(current_gap) > len(max_gap):
                    max_gap = current_gap.copy()
                current_gap = []

        if len(current_gap) > len(max_gap):
            max_gap = current_gap

        self.max_distance_index = max_gap[int(len(max_gap)//2)]

        self.get_logger().info(
            f"Max distance index: {self.max_distance_index}, "
            f"Distance: {msg.ranges[self.max_distance_index]}"
        )

        self.cmd_pos_ctrl(msg, self.max_distance_index)

    def cmd_pos_ctrl(self, msg, max_distance_index):
        self.x = msg.ranges[max_distance_index]
        if self.x == float('inf'):
            self.x = msg.range_max
        self.angle = msg.angle_min + (max_distance_index * msg.angle_increment)

    def timer_callback(self):
        twist_msg = Twist()

        if self.x > 2.0:
            twist_msg.linear.x = 0.8
        elif self.x > 1.0:
            twist_msg.linear.x = 0.5
        else:
            twist_msg.linear.x = 0.2

        # Mapear velocidad lineal
        twist_msg.linear.x = self.map(twist_msg.linear.x, 0.0, 0.8, -0.5, 0.5)

        # Ángulo relativo
        twist_msg.angular.z = self.angle
        self.cmd_pos_ctrl_pub.publish(twist_msg)

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def main(args=None):
    rclpy.init(args=args)
    node = GapFinderCharlie()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
