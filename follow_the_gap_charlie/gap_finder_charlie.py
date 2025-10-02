#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class GapFinderCharlie(Node): 
    def __init__(self):
        super().__init__("gap_finder_charlie_node")
        self.scan_sub = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.cmd_pos_ctrl_pub = self.create_publisher(Twist, '/cmd_pos_ctrl', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.r = 20
        self.gap_threshold = 2.0
        self.n = 40
        self.max_distance_index = 0
        self.x = 0.0
        self.angle = 0.0

    def get_front_ranges(self, msg):
        ranges_array = np.array(msg.ranges)

        n = len(ranges_array)
        angle_increment = (msg.angle_max - msg.angle_min) / n

        # Número de índices que corresponden a 90°
        idx_90 = int((np.pi/2) / angle_increment)

        # Primer sector: 0°–90°
        sector_1 = ranges_array[0:idx_90]

        # Segundo sector: 270°–360°
        sector_2 = ranges_array[-idx_90:]

        # Concatenar → representa -90° a +90°
        front_ranges = np.concatenate((sector_2, sector_1))

        return front_ranges

    def get_Gaps(self, msg):
        indexes = []

        ranges_array = self.get_front_ranges(msg)

        # Encontrar el índice del valor mínimo
        laser_min_index = np.argmin(ranges_array)

        # Crear copia para no modificar original
        ranges_copy = ranges_array.copy()

        # Aplicar slicing alrededor del obstáculo más cercano
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

        # Encontrar la brecha más larga
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

        # Calcular la distancia y ángulo relativo
        front_ranges = self.get_front_ranges(msg)
        self.x = front_ranges[self.max_distance_index]
        if self.x == float('inf'):
            self.x = msg.range_max

        # Ángulo relativo en [-pi/2, +pi/2]
        self.angle = -np.pi/2 + (self.max_distance_index * (np.pi / len(front_ranges)))

        self.get_logger().info(
            f"Max distance index: {self.max_distance_index}, "
            f"Distance: {self.x:.2f}, Angle: {np.degrees(self.angle):.1f}°"
        )

    def timer_callback(self):
        twist_msg = Twist()

        # Velocidad base según distancia
        if self.x > 2.0:
            twist_msg.linear.x = 0.75
        elif self.x > 1.0:
            twist_msg.linear.x = 0.6
        else:
            twist_msg.linear.x = 0.5

        # Mapear velocidad lineal
        twist_msg.linear.x = self.map(twist_msg.linear.x, 0.0, 0.8, -0.5, 0.5)

        # Penalizar velocidad si el ángulo es grande
        # factor = 1.0 cuando angle = 0°, y se reduce hasta ~0.2 cuando angle → ±90°
        # factor = max(0.2, 1 - abs(self.angle) / (np.pi / 2))
        # twist_msg.linear.x *= factor

        # Ángulo relativo (ya calculado en [-pi/2, pi/2])
        twist_msg.angular.z = self.map(self.angle, -np.pi/2, np.pi/2, -0.5, 0.5)

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
