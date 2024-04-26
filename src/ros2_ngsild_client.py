#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import math
from ngsildclient import Entity, Client, SubscriptionBuilder, SmartDataModels

class ROS2NGSILDClient(Node):
    
    # Class variables
    input_mission_ = GeoPath()
    tractor_gnss_ = GeoPoseStamped()
    tractor_map_pose_ = Pose()

    

    def __init__(self):
        super().__init__('ros2_ngsild_client')

        self.AUTONOMOUS_MOBILE_ROBOT_CONTEXT = "https://raw.githubusercontent.com/smart-data-models/dataModel.AutonomousMobileRobot/master/context.jsonld"

        #Get node parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('tractor_position_topic', '/sd_tractor/position'),
                ('mission_topic', '/sd_tractor/mission'),
            ])
        self.tractor_position_topic = self.get_parameter('tractor_position_topic').value
        self.mission_topic = self.get_parameter('mission_topic').value
        
        # Create ROS publishers and subscribers
        self.input_mission_publisher_ = self.create_publisher(GeoPath, self.mission_topic, 10)
        self.tractor_position_subscriber_ = self.create_subscription(
            GeoPoseStamped,
            self.tractor_position_topic,
            self.tractor_position_callback,
            10)

        # Timer for feedback publishing
        self.command_message_timer_ = self.create_timer(1, self.command_timer_callback)
        self.state_message_timer_ = self.create_timer(5, self.state_timer_callback)

        # Initialized SmartDataModels messages
        self.command_message_ = Entity("CommandMessage","MissionCommand")
        self.state_message_= Entity.load(f"https://smart-data-models.github.io/dataModel.Agrifood/AgriFarm/examples/example-normalized.jsonld")
        self.state_message_.id = f'urn:ngsi-ld:StateMessage:Status-01'

        # Initialized variables
        self.first_state_message_ = True

        self.get_logger().info('[ROS2_NGSILD_CLIENT] Initialized')
    
    # def create_entity_subscription(self, entity):
    #     with Client() as client:
            

    def input_mission_callback(self, msg):
        self.get_logger().info('[ROS2_NGSILD_CLIENT] Received mission from broker')
        # TBD: Transform from received NGSILD mission to ROS2 geographic_msgs::GeoPath 
        # input_mission_ = convertFromNGSIDL()
        self.input_mission_publisher_.publish(ROS2NGSILDClient.input_mission_)

    def tractor_position_callback(self, msg):
        self.get_logger().debug('[ROS2_NGSILD_CLIENT] Tractor position cb')
        # TBD by EUT: Get lat,lon and map positioning of tractor from it
        # tractor_gnss_ = ...
        # tractor_map_pose_ = ...

    def command_timer_callback(self):
        with Client() as client: 
            try:
                self.get_logger().debug('[ROS2_NGSILD_CLIENT] Requesting any CommandMessage in OCB')
                # self.command_message_ = client.get("CommandMessage:Mission", ctx=)
                # client.get(self.state_message_) 
            except: 
                self.get_logger().debug('[ROS2_NGSILD_CLIENT] There was no StateMessage in OCB')
    
    def state_timer_callback(self):
        with Client() as client: 
          try:
              self.get_logger().warn('[ROS2_NGSILD_CLIENT] First StateMessage in OCB')
              client.create(self.state_message_)    
          except:
              self.get_logger().warn('[ROS2_NGSILD_CLIENT] Updating StateMessage in OCB')
              client.update(self.state_message_)



def main():
    rclpy.init()
    node = ROS2NGSILDClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()