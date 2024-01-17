#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import math
import yaml
import os
import pathlib

class LaserScanFilterNode(Node):
    def __init__(self, config_file):
        super().__init__('laser_scan_filter_node')
        self.load_config(config_file)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_scan_callback,
            qos_profile)

        self.publisher = self.create_publisher(
            LaserScan,
            'scan_filtered',
            qos_profile)

    def load_config(self, config_file):
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
            filter_config = config['scan_to_scan_filter_chain']['ros__parameters']['filter1']['params']
            self.lower_bound = filter_config.get('lower_angle', -math.pi)  # Default to -180 degrees
            self.upper_bound = filter_config.get('upper_angle', math.pi)   # Default to 180 degrees

    def laser_scan_callback(self, msg):
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header

        # Filtering logic
        filtered_ranges = []
        for angle, distance in enumerate(msg.ranges):
            current_angle = msg.angle_min + angle * msg.angle_increment
            if self.lower_bound <= current_angle <= self.upper_bound:
                filtered_ranges.append(distance)
            else:
                filtered_ranges.append(float('inf'))  # or some other value indicating no reading

        filtered_scan.ranges = filtered_ranges
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max

        # Publish the filtered scan
        self.publisher.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)

    config_file_path = str(pathlib.Path(__file__).parent.absolute().joinpath('config', 'laser_filter.yaml'))

    laser_scan_filter_node = LaserScanFilterNode(config_file_path)
    rclpy.spin(laser_scan_filter_node)
    laser_scan_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()