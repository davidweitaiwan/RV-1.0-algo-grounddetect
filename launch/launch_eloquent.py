#ver=1.2
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
            node_namespace=data['generic_prop']['namespace'],
            node_executable="zed",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "topic_ZEDCam_RGB_nodeName" : data['topic_ZEDCam_RGB']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "topic_ZEDCam_RGB_topicName" : data['topic_ZEDCam_RGB']['topicName'], 
                    "topic_ZEDCam_Depth_nodeName" : data['topic_ZEDCam_Depth']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "topic_ZEDCam_Depth_topicName" : data['topic_ZEDCam_Depth']['topicName'], 
                    "topic_GroundDetect_nodeName" : data['topic_GroundDetect']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "topic_GroundDetect_topicName" : data['topic_GroundDetect']['topicName'] + '_' + str(data['generic_prop']['id']), 
                    "mainCameraWidth" : data['camera_prop']['width'], 
                    "mainCameraHeight" : data['camera_prop']['height'], 

                    # Settings for Params class under vehicle_interfaces/params.h
                    # Do not change the settings rashly
                    "nodeName" : data['generic_prop']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "id" : data['generic_prop']['id'], 
                    "devInfoService" : data['generic_prop']['devInfoService'], 
                    "devInterface" : data['generic_prop']['devInterface'], 
                    "qosService" : data['generic_prop']['qosService'], 
                    "qosDirPath" : data['generic_prop']['qosDirPath'], 
                    "safetyService" : data['generic_prop']['safetyService'], 
                    "timesyncService" : data['generic_prop']['timesyncService'], 
                    "timesyncPeriod_ms" : data['generic_prop']['timesyncPeriod_ms'], 
                    "timesyncAccuracy_ms" : data['generic_prop']['timesyncAccuracy_ms'], 
                }
            ]
        )
    ])
