#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import math
from ngsildclient import Entity, Client, SubscriptionBuilder, SmartDataModels

from retrofitted_tractor_data_adapters.msg import CommandMessage, StateMessage, GeographicPose
from retrofitted_tractor_data_adapters.srv import NGSILDFile

import sd_data_adapter.models.agri_food as models
from sd_data_adapter.client import DAClient
from sd_data_adapter.api import upload

class ROS2NGSILDClient(Node):
       
    def __init__(self):
        super().__init__('ros2_ngsild_client')

        self.AUTONOMOUS_MOBILE_ROBOT_CONTEXT = "https://raw.githubusercontent.com/smart-data-models/dataModel.AutonomousMobileRobot/master/context.jsonld"
      
        # Create ROS publishers and subscribers
        
        # Create service servers
        self.send_ngsild_message_srv = self.create_service(NGSILDFile, "/send_ngsild_message", self.send_ngsild_message_callback)

        # Create service clients
        self.convert_ngsild_to_ros2_srv = self.create_client(NGSILDFile, "/sdm_to_ros2")
        while not self.convert_ngsild_to_ros2_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Initialized SmartDataModels messages
        self.command_message_ = Entity.load(f"https://smart-data-models.github.io/dataModel.AutonomousMobileRobot/CommandMessage/examples/example-geopoint.json")
        self.state_message_= Entity.load(f"https://smart-data-models.github.io/dataModel.Agrifood/AgriFarm/examples/example-normalized.jsonld")
        
        # Create subscription to CommandMessage entity
        # self.create_command_message_subscription() -> has to callback self.command_message_callback()

        # Initialized variables
        self.get_logger().info('[ROS2_NGSILD_CLIENT] Initialized')
    

    def command_message_callback(self):
        self.get_logger().info('[ROS2_NGSILD_CLIENT] Received CommandMessage from broker')
        # TBD: Store received message in a json file
        # command_message_file = client.get(....) 
        request = NGSILDFile.Request()
        request.message_type = "CommandMessage"
        request.file_path = "path/to/command_message_file"

        future = self.convert_ngsild_to_ros2_srv.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().debug("'[ROS2_NGSILD_CLIENT] Converted NGSILD to ROS2")
        else:
            self.get_logger().warn(" '[ROS2_NGSILD_CLIENT] NGSILD to ROS2 conversion failed!")

    def send_ngsild_message_callback(self, request, response):
        
        if (request.message_type != "StateMessage"):
          self.get_logger().warn("'[ROS2_NGSILD_CLIENT] Received wrong StateMessage!")
          response.success = False
          return response
                
        state_message_file_path = request.file_path

        #TBD: From file path provided, load JSON and send it to broker 
        # e = Entity.load(state_message_file_path)
        # client.send(e)

        self.get_logger().debug("'[ROS2_NGSILD_CLIENT] Sent StateMessage from JSON file")
        response.success = True

        return response

def main():
    rclpy.init()
    node = ROS2NGSILDClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()