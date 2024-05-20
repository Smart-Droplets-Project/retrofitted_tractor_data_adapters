#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool, Float32, Header

from retrofitted_tractor_data_adapters.msg import CommandMessage, StateMessage, GeographicPose
from retrofitted_tractor_data_adapters.srv import NGSILDFile

import json
import math

class SDMToROS2(Node):
    def __init__(self):
        super().__init__('sdm_to_ros2')

        self.to_ros2_service = self.create_service(NGSILDFile, "/sdm_to_ros2", self.translate_to_ros2_callback)

        self.command_message_publisher = self.create_publisher(
            CommandMessage,
            "/sd_tractor/mission",
            10
        )  

        self.state_message_publisher = self.create_publisher(
            StateMessage,
            "/sd_tractor/status",
            10
        )  

        self.get_logger().info('[SDMToROS2] Initialized')

    def translate_to_ros2_callback(self, request, response):
        try:
            with open(request.file_path, 'r') as file:
                data = json.load(file)
            
            print("JSON Data:")
            print(json.dumps(data, indent=4))
            
            if request.message_type == 'CommandMessage':
                self.command_message_to_ros2(data)
                response.success = True
            elif request.message_type == 'StateMessage':
                self.state_message_to_ros2(data)
                response.success = True
            else:
                self.get_logger().warn('[SDMToROS2] Unsupported message type: %s', request.message_type)
                response.success = False
        except Exception as e:
            self.get_logger().error('[SDMToROS2] Error while translating JSON: %s', str(e))
            response.success = False

        return response

    def read_json_file(self,file_path):
        with open(file_path, 'r') as file:
            data = json.load(file)
        return data
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw
    
    def command_message_to_ros2(self, data):
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
    
        command_message_msg = CommandMessage()

        command_message_msg.header = header
        command_message_msg.command = str(data['command'])
        command_message_msg.command_time = str(data['commandTime'])
        command_message_msg.type = str(data['type'])
        
        waypoints = data['waypoints']
        for index, waypoint in enumerate(waypoints):
            geographic_pose_msg = GeographicPose()
            geographic_pose_msg.geographic_point.latitude = waypoint['geographicPoint']['latitude']
            geographic_pose_msg.geographic_point.longitude = waypoint['geographicPoint']['longitude']
            geographic_pose_msg.geographic_point.altitude = waypoint['geographicPoint']['altitude']
            
            quaternion_msg = Quaternion()
            
            if 'orientation3D' in waypoint:
                orientation_3d = waypoint['orientation3D']
                quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w = self.euler_to_quaternion(orientation_3d['roll'], orientation_3d['pitch'], orientation_3d['yaw'])
            
            geographic_pose_msg.orientation_3d = quaternion_msg
            command_message_msg.waypoints.append(geographic_pose_msg)
        
        self.command_message_publisher.publish(command_message_msg)
      
    def state_message_to_ros2(self, data):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
    
        state_message_msg = StateMessage()

        state_message_msg.header = header
        state_message_msg.command_time = str(data['commandTime'])
        state_message_msg.type = str(data['type'])
        state_message_msg.mode = str(data['mode'])

        errors = data['errors']
        for index, error in enumerate(errors):
            state_message_msg.errors.append(error)
        
        pose = data['pose']
        
        pose_msg = GeographicPose()
        pose_msg.geographic_point.latitude = pose['geographicPoint']['latitude']
        pose_msg.geographic_point.longitude = pose['geographicPoint']['longitude']
        pose_msg.geographic_point.altitude = pose['geographicPoint']['altitude']

        orientation_3d = pose['orientation3D']
        quaternion_msg = Quaternion()
        quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w = self.euler_to_quaternion(orientation_3d['roll'], orientation_3d['pitch'], orientation_3d['yaw'])
        pose_msg.orientation_3d = quaternion_msg

        state_message_msg.pose = pose_msg

        destination = data['destination']
        
        destination_msg = GeographicPose()
        destination_msg.geographic_point.latitude = destination['geographicPoint']['latitude']
        destination_msg.geographic_point.longitude = destination['geographicPoint']['longitude']
        destination_msg.geographic_point.altitude = destination['geographicPoint']['altitude']

        orientation_3d = destination['orientation3D']
        quaternion_msg = Quaternion()
        quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w = self.euler_to_quaternion(orientation_3d['roll'], orientation_3d['pitch'], orientation_3d['yaw'])
        destination_msg.orientation_3d = quaternion_msg

        state_message_msg.destination = destination_msg

        accuracy = data['accuracy']['covariance']
        for index, covariance in enumerate(accuracy):
            state_message_msg.accuracy.append(covariance)

        state_message_msg.battery = float(data['battery']['remainingPercentage'])

        self.state_message_publisher.publish(state_message_msg)        

def main(args=None):
    rclpy.init(args=args)
    node = SDMToROS2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()