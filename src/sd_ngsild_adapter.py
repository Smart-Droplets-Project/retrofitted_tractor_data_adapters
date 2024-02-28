#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import math

class NDSILDAdapter(Node):
    
    # Class variables
    input_mission_ = GeoPath()
    tractor_gnss_ = GeoPoseStamped()
    tractor_map_pose_ = Pose()

    def __init__(self):
        super().__init__('sd_ngsild_adapter')

        #Get node parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('tractor_position_topic', '/sd_tractor/position'),
                ('mission_topic', '/sd_tractor/mission'),
                ('feedback_sender_frequency', 10.0)  
            ])
        self.tractor_position_topic = self.get_parameter('tractor_position_topic').value
        self.mission_topic = self.get_parameter('mission_topic').value
        self.feedback_sender_frequency = self.get_parameter('feedback_sender_frequency').value
        
        # Create ROS publishers and subscribers
        self.input_mission_publisher_ = self.create_publisher(GeoPath, self.mission_topic, 10)
        self.tractor_position_subscriber_ = self.create_subscription(
            GeoPoseStamped,
            self.tractor_position_topic,
            self.tractor_position_callback,
            10)

        # Timer for feedback publishing
        self.timer_ = self.create_timer(1.0 / self.feedback_sender_frequency, self.timer_callback)

        # TBD: Function for initialize communication with context broker
        # initializeBrokerCommunication()

        # TBD: Configure reception events for input. Set it so that, whenever received, it calls function input_mission_callback()
        # configureCommunicationEvents()

        self.get_logger().info('[SD_NGSILD_ADAPTER] Initialized')

    def input_mission_callback(self, msg):
        self.get_logger().info('[SD_NGSILD_ADAPTER] Received mission from broker')
        # TBD: Transform from received NGSILD mission to ROS2 geographic_msgs::GeoPath 
        # input_mission_ = convertFromNGSIDL()
        self.input_mission_publisher_.publish(NDSILDAdapter.input_mission_)

    def tractor_position_callback(self, msg):
        self.get_logger().debug('[SD_NGSILD_ADAPTER] Tractor position cb')
        # TBD by EUT: Get lat,lon and map positioning of tractor from it
        # tractor_gnss_ = ...
        # tractor_map_pose_ = ...

    def timer_callback(self):
        self.get_logger().debug('[SD_NGSILD_ADAPTER] Timer cb')
        # TBD: From all information received in feedback and stored as ROS2 msgs, now convert this to format
        #      and send it to broker.
        # ngslid_msg = convertToNGSILD()
        # sendNGSLIDFeedback(ngslid_msg)

def main():
    rclpy.init()
    node = NDSILDAdapter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()