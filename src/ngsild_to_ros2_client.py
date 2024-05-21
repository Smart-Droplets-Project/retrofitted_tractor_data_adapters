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
    
        self.declare_parameter('subscription.list', '')
        subscription_list_param = self.get_parameter('subscription.list').get_parameter_value().string_value
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

        # Initialized variables
        self.get_logger().info('[NGSILD_ROS2_CLIENT] Initialized')

    def entity_get_callback(self):
        for id in self.subscription_list:
            try: 
              self.model_to_ros2(get_by_id(id))
            except: 
              self.get_logger().info(f'No model found with id {id}')

    def model_to_ros2(self, model):
        if (model.type == 'CommandMessage'):
            self.parse_command_message(model)

    def parse_command_message(self, command_message):
        ros2_command_message = CommandMessage()
        ros2_command_message.header.stamp = self.get_clock().now().to_msg()
        ros2_command_message.command = command_message.command
        ros2_command_message.command_time = command_message.commandTime
        ros2_command_message.type = command_message.type
        for waypoint in command_message.waypoints:
            ros2_waypoint = GeographicPose()
            ros2_geographic_point = GeoPoint()
            ros2_geographic_point.latitude = float(waypoint['geographicPoint']['latitude'])
            ros2_geographic_point.longitude = float(waypoint['geographicPoint']['longitude'])
            ros2_geographic_point.altitude = float(waypoint['geographicPoint']['altitude'])
            ros2_waypoint.geographic_point = ros2_geographic_point
            q = euler2quat(float(waypoint['orientation3D']['roll']), float(waypoint['orientation3D']['pitch']), float(waypoint['orientation3D']['yaw']))
            ros2_quaternion = Quaternion()
            ros2_quaternion.x = q[1]
            ros2_quaternion.y = q[2]
            ros2_quaternion.z = q[3]
            ros2_quaternion.w = q[0]
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