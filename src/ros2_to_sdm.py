#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from retrofitted_tractor_data_adapters.msg import CommandMessage, StateMessage, GeographicPose
from rclpy.parameter import Parameter
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool, Float32, Header

import json
import math
from math import atan2, asin
from datetime import datetime

class ROS2ToSDM(Node):
    def __init__(self):
        super().__init__('ros2_to_sdm')

        self.declare_parameter('robot_id', 'eutrob:01')
        self.declare_parameter('command_message_topic', '/sdm/command_message')
        self.declare_parameter('state_message_topic', '/sdm/state_message')

        self.robot_id = self.get_parameter('robot_id').value
        command_message_topic = self.get_parameter('command_message_topic').value
        state_message_topic = self.get_parameter('state_message_topic').value

        self.command_message_sub = self.create_subscription(
            CommandMessage,
            command_message_topic,
            self.command_message_callback,
            10
        )

        self.state_message_sub = self.create_subscription(
            StateMessage,
            state_message_topic,
            self.state_message_callback,
            10
        )

    def command_message_callback(self, msg):
        json_data = self.command_message_to_json(msg)
        json_str = json.dumps(json_data, indent=4)
        self.get_logger().info(f"Received Command Message:\n{json_str}")
    
    def state_message_callback(self, msg):
        json_data = self.state_message_to_json(msg)
        json_str = json.dumps(json_data, indent=4)
        self.get_logger().info(f"Received State Message:\n{json_str}")
    
    def quaternion_to_euler(self, quaternion):
        roll = atan2(2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),1 - 2 * (quaternion.x ** 2 + quaternion.y ** 2))
        pitch = asin(2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x))
        yaw = atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2))
        return roll, pitch, yaw
    
    def command_message_to_json(self, command_message_msg):
        
        data = {}

        data['id'] = self.robot_id
        data['command'] = command_message_msg.command
        data['commandTime'] = datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "+09:00"
        data['type'] = command_message_msg.type

        waypoints_data = []
        for waypoint in command_message_msg.waypoints:
            waypoint_data = {
                'geographicPoint': {
                    'latitude': waypoint.geographic_point.latitude,
                    'longitude': waypoint.geographic_point.longitude,
                    'altitude': waypoint.geographic_point.altitude
                }
            }

            if waypoint.orientation_3d:
                quaternion_msg = waypoint.orientation_3d
                roll, pitch, yaw = self.quaternion_to_euler(quaternion_msg)
                waypoint_data['orientation3D'] = {
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw
                }

            waypoints_data.append(waypoint_data)

        data['waypoints'] = waypoints_data

        return data
    
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

        return data

def main(args=None):
    rclpy.init(args=args)
    node = ROS2ToSDM()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()