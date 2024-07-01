import rclpy
import numpy
import os
import csv
import pandas as pd
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class Subscriber(Node):

    def __init__(self):
        super().__init__('watchtower')
        # Define subscriber
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            10)
        # Define publisher
        self.publisher_ = self.create_publisher(Float32, 'map_progress', 10)
        self.free_thresh = 0.25
        self.percentage_explored = 0.0

    def check_map_completion(self):
        if self.map_data is None:
            return
        
        # Überprüfe die Kartendaten
        unexplored_cells = sum(1 for cell in self.map_data.data if cell == -1)
        total_cells = len(self.map_data.data)
        explored_cells = total_cells - unexplored_cells
        self.get_logger().info('Unexplored cells: %s, total cells: %s' %(unexplored_cells, total_cells))

        self.percentage_explored = explored_cells / total_cells

        # Definiere einen Schwellenwert für die Vollständigkeit der Karte
        if unexplored_cells < (total_cells * 0.01):  # z.B. weniger als 1% der Karte ist unerforscht
            self.get_logger().info('Die gesamte Karte wurde erfolgreich aufgezeichnet.')

    def listener_callback(self, msg):
        self.map_data = msg
        self.check_map_completion()
        percentage_explored = self.percentage_explored
        map_explored_msg = Float32()
        if percentage_explored > 1.0:
            percentage_explored = 1.0
        map_explored_msg.data = self.percentage_explored
        self.publisher_.publish(map_explored_msg)

def main(args=None):
    rclpy.init(args=args)

    watchtower = Subscriber()

    rclpy.spin(watchtower)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    watchtower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()