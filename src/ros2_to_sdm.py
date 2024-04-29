#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool, Float32, Header

import json
import os
import math
from math import atan2, asin
from datetime import datetime

from retrofitted_tractor_data_adapters.msg import CommandMessage, StateMessage, GeographicPose
from retrofitted_tractor_data_adapters.srv import NGSILDFile

class ROS2ToSDM(Node):
    def __init__(self):
        super().__init__('ros2_to_sdm')

        self.package_path = os.path.dirname(__file__)

        self.json_folder_path = os.path.join(self.package_path, '../json')

        self.declare_parameter('robot_id', 'eutrob:01')
        self.declare_parameter('state_message_topic', '/state_message')
        self.declare_parameter('send_ngsild_message_srv_name', '/send_ngsild_message')

        self.robot_id = self.get_parameter('robot_id').value
        command_message_topic = self.get_parameter('command_message_topic').value
        state_message_topic = self.get_parameter('state_message_topic').value
        send_ngsild_message_srv_name = self.get_parameter('send_ngsild_message_srv_name').value

        self.state_message_sub = self.create_subscription(
            StateMessage,
            state_message_topic,
            self.state_message_callback,
            10
        )

        self.send_ngsild_message_srv_name = self.create_client(NGSILDFile, self.convert_sdm_to_ros2_srv_name)
        while not self.send_ngsild_message_srv_name.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('[ROS2ToSDM] Service not available, waiting again...')

        self.get_logger().info('[ROS2ToSDM] Initialized')
    
    def state_message_callback(self, msg):
        json_data = self.state_message_to_json(msg)
    
    def quaternion_to_euler(self, quaternion):
        roll = atan2(2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),1 - 2 * (quaternion.x ** 2 + quaternion.y ** 2))
        pitch = asin(2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x))
        yaw = atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2))
        return roll, pitch, yaw
    
    def save_data_to_json(self, filepath, data):
        with open(filepath, 'w') as json_file:
            json.dump(data, json_file, indent=4)
       
    def state_message_to_json(self, state_message_msg):
        
        data = {}

        data['id'] = self.robot_id
        data['commandTime'] = datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "+09:00"
        data['type'] = state_message_msg.type
        data['errors'] = [str(error) for error in state_message_msg.errors]

        pose_data = state_message_msg.pose
        quaternion_msg = pose_data.orientation_3d
        roll, pitch, yaw = self.quaternion_to_euler(quaternion_msg)
        pose_json = {
            'geographicPoint': {
                'latitude': pose_data.geographic_point.latitude,
                'longitude': pose_data.geographic_point.longitude,
                'altitude': pose_data.geographic_point.altitude
            },
            'orientation3D': {
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw
            }
        }
        data['pose'] = pose_json

        destination_data = state_message_msg.pose
        quaternion_msg = destination_data.orientation_3d
        roll, pitch, yaw = self.quaternion_to_euler(quaternion_msg)
        destination_json = {
            'geographicPoint': {
                'latitude': destination_data.geographic_point.latitude,
                'longitude': destination_data.geographic_point.longitude,
                'altitude': destination_data.geographic_point.altitude
            },
            'orientation3D': {
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw
            }
        }
        data['destination'] = destination_json

        data['accuracy'] = {
         'covariance': [float(x) for x in state_message_msg.accuracy]
        }
        
        data['battery'] = {
        'remainingPercentage': state_message_msg.battery
        }

        filename = "state_message.json"
        filepath = os.path.join(self.json_folder_path, filename)
        self.save_data_to_json(filepath, data)
        self.get_logger().debug('[ROS2ToSDM] Stored JSON file')

        request = NGSILDFile.Request()
        request.message_type = "StateMessage"
        request.file_path = filepath
        
        future = self.send_ngsild_message_srv_name.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().debug("'[ROS2ToSDM] Sent NGSILD message from ROS2")
        else:
            self.get_logger().warn(" '[ROS2ToSDM] NGSILD message from ROS2 transmission failed!")

        return data

def main(args=None):
    rclpy.init(args=args)
    node = ROS2ToSDM()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()