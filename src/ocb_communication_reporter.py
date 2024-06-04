#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geographic_msgs.msg import GeoPath, GeoPoseStamped, GeoPoint
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Float64, Empty
from transforms3d.euler import euler2quat

from ngsildclient import Entity, Client, SubscriptionBuilder, SmartDataModels

from retrofitted_tractor_data_adapters.msg import CommandMessage, StateMessage, GeographicPose

import datetime
import sd_data_adapter.models as models
from sd_data_adapter.client import DAClient
from sd_data_adapter.api import upload, get_by_id, search

class OCBCommunicationReporter(Node):
       
    def __init__(self):
        super().__init__('ocb_communication_reporter')

        # Get ROS parameters
        self.declare_parameter('client.host', 'localhost')
        self.declare_parameter('client.port', 1026)
        host = self.get_parameter('client.host').value
        port = self.get_parameter('client.port').value
    
        self.declare_parameter('check.frequency', 10.0)
        check_freq = self.get_parameter('check.frequency').value
        
        # Create ROS publishers and subscribers
        self.ocb_communication_pub = self.create_publisher(Empty, 'ocb_communication', 1)

        # Create timer for communication checking
        self.entity_get_timer = self.create_timer(1/check_freq, self.communication_callback)

        # Set up Context Broker Client
        DAClient.get_instance(host, port)

        # Initialized variables
        self.get_logger().info('[OCB Communication Reporter] Initialized')

    def communication_callback(self):
        search_params = {}
        result = Empty()
        try:
          models = search(search_params)
          self.ocb_communication_pub.publish(result)
        except:
          self.get_logger().debug('[OCB Communication Reporter] No communication')
        

def main():
    rclpy.init()
    node = OCBCommunicationReporter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()