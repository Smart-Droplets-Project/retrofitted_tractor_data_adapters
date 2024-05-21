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
from sd_data_adapter.api import upload, get_by_id, update

class ROS2ToNGSILDClient(Node):
       
    def __init__(self):
        super().__init__('ros2_to_ngsild_client')

        # Get ROS parameters
        self.declare_parameter('client.host', 'localhost')
        self.declare_parameter('client.port', 1026)
        host = self.get_parameter('client.host').value
        port = self.get_parameter('client.port').value

        self.declare_parameter('publishment.frequency', 10.0)
        publishment_freq = self.get_parameter('publishment.frequency').value

        self.declare_parameter('publishment.state_message.id', "StateMessage:0001-id")
        self.state_message_id = self.get_parameter('publishment.state_message.id').value
        
        # Create ROS publishers and subscribers
        self.state_message_sub = self.create_subscription(StateMessage,'/state_message',self.state_message_callback,10)

        # Create timer for Entity checking
        self.entity_update_timer = self.create_timer(1/publishment_freq, self.entity_update_callback)

        # Set up Context Broker Client
        DAClient.get_instance(host, port)

        self.state_message_ = StateMessage()

        # Initialized variables
        self.get_logger().info('[ROS2_TO_NGSILD_CLIENT] Initialized')

    def state_message_callback(self, msg):
        self.state_message_ = msg

    def entity_update_callback(self):
        self.send_state_message(self.state_message_)

    def send_state_message(self, ros2_state_message):
        model = models.autonomous_mobile_robot.StateMessage(self.state_message_id)
        model.battery = ros2_state_message.battery
        model.commandTime = ros2_state_message.command_time
        model.mode = ros2_state_message.mode
        try:
            update(model) 
        except:
            upload(model)
        self.get_logger().info('[ROS2_TO_NGSILD_CLIENT] Uploaded state message')


def main():
    rclpy.init()
    node = ROS2ToNGSILDClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()