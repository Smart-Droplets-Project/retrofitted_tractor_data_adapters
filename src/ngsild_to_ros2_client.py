#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geographic_msgs.msg import GeoPath, GeoPoseStamped, GeoPoint
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Float64
from transforms3d.euler import euler2quat

from ngsildclient import Entity, Client, SubscriptionBuilder, SmartDataModels

from retrofitted_tractor_data_adapters.msg import CommandMessage, StateMessage, GeographicPose

import datetime
import sd_data_adapter.models as models
from sd_data_adapter.client import DAClient
from sd_data_adapter.api import upload, get_by_id

class NGSILDToROS2Client(Node):
       
    def __init__(self):
        super().__init__('ngsild_to_ros2_client')

        # Get ROS parameters
        self.declare_parameter('client.host', 'localhost')
        self.declare_parameter('client.port', 1026)
        host = self.get_parameter('client.host').value
        port = self.get_parameter('client.port').value

        self.get_logger().info(f'Client host: {host}')
        self.get_logger().info(f'Client port: {port}')
    
        self.declare_parameter('subscription.list', '')
        subscription_list_param = self.get_parameter('subscription.list').value
        if subscription_list_param:
            self.subscription_list = subscription_list_param.split(',')
        else:
            self.subscription_list = []
        self.get_logger().info(f'Subscription ids: {self.subscription_list}')

        self.declare_parameter('subscription.frequency', 10.0)
        subscription_freq = self.get_parameter('subscription.frequency').value
        
        # Create ROS publishers and subscribers
        self.command_message_pub = self.create_publisher(CommandMessage, 'command_message', 10)

        # Create timer for Entity checking
        self.entity_get_timer = self.create_timer(1/subscription_freq, self.entity_get_callback)

        # Set up Context Broker Client
        DAClient.get_instance(host, port)

        # Initialize variables
        self.last_command_time = ""

        self.get_logger().info('[NGSILD_ROS2_CLIENT] Initialized')

    def entity_get_callback(self):
        for id in self.subscription_list:
            try: 
              self.model_to_ros2(get_by_id(id))
            except: 
              self.get_logger().debug(f'No model found with id {id}')

    def model_to_ros2(self, model):
        if model.type.strip() == 'CommandMessage':
            self.parse_command_message(model)

    def parse_command_message(self, command_message):
        ros2_command_message = CommandMessage()
        ros2_command_message.header.stamp = self.get_clock().now().to_msg()
        ros2_command_message.command = command_message.command
        ros2_command_message.command_time = command_message.commandTime
        if ros2_command_message.command_time == self.last_command_time:
            self.get_logger().debug(f'Command message already processed with stamp {ros2_command_message.command_time}')
            return
        self.last_command_time = ros2_command_message.command_time
        ros2_command_message.type = command_message.type
        for waypoint in command_message.waypoints:
            ros2_waypoint = GeographicPose()
            ros2_geographic_point = GeoPoint()
            ros2_geographic_point.latitude = float(waypoint['geographicPoint']['latitude'])
            ros2_geographic_point.longitude = float(waypoint['geographicPoint']['longitude'])
            ros2_geographic_point.altitude = float(waypoint['geographicPoint']['altitude'])
            ros2_waypoint.geographic_point = ros2_geographic_point
            ros2_quaternion = Quaternion()
            ros2_waypoint.orientation_3d = ros2_quaternion
            ros2_command_message.waypoints.append(ros2_waypoint)

        self.command_message_pub.publish(ros2_command_message)

def main():
    rclpy.init()
    node = NGSILDToROS2Client()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()