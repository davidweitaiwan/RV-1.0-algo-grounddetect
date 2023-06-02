from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml
from yaml import load, Loader

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory('cpp_zeddetect'), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)
    return LaunchDescription([
        Node(
            package="cpp_zeddetect",
            node_namespace=data['node_prop']['namespace'],
            node_executable="zed",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "topic_ZEDCam_RGB_nodeName" : data['topic_ZEDCam_RGB']['nodeName'], 
                    "topic_ZEDCam_RGB_topicName" : data['topic_ZEDCam_RGB']['topicName'], 
                    "topic_ZEDCam_Depth_nodeName" : data['topic_ZEDCam_Depth']['nodeName'], 
                    "topic_ZEDCam_Depth_topicName" : data['topic_ZEDCam_Depth']['topicName'], 
                    "topic_GroundDetect_nodeName" : data['topic_GroundDetect']['nodeName'], 
                    "topic_GroundDetect_topicName" : data['topic_GroundDetect']['topicName'], 
                    "service_SafetyServer_serviceName" : data['service_SafetyServer']['serviceName'], 
                    "mainNodeName" : data['node_prop']['nodeName'], 
                    "mainNamespace" : data['node_prop']['namespace'], 
                    "mainCameraWidth" : data['camera_prop']['width'], 
                    "mainCameraHeight" : data['camera_prop']['height'], 
                }
            ]
        )
    ])
