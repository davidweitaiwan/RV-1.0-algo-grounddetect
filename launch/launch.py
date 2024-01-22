#ver=2.0
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import json
import yaml
from yaml import load, Loader

pkgName = 'cpp_zeddetect'

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)

    serviceFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/service.json')
    with open(serviceFilePath, 'r') as f:
        serviceData = json.load(f)

    return LaunchDescription([
        Node(
            package=pkgName,
            namespace=data['generic_prop']['namespace'],
            executable="zed",
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
                    "safetyDirection" : data['safety_prop']['direction'], 
                    "safetyDistance" : data['safety_prop']['alert_distance_mm'], 

                    # Settings for Params class under vehicle_interfaces/params.h
                    # Do not change the settings rashly
                    "nodeName" : data['generic_prop']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "id" : data['generic_prop']['id'], 
                    "devInfoService" : serviceData['devInfoService'], 
                    "devInterface" : serviceData['devInterface'], 
                    "devMultiNode" : serviceData['devMultiNode'], 
                    "qosService" : serviceData['qosService'], 
                    "qosDirPath" : serviceData['qosDirPath'], 
                    "safetyService" : serviceData['safetyService'], 
                    "timesyncService" : serviceData['timesyncService'], 
                    "timesyncPeriod_ms" : serviceData['timesyncPeriod_ms'], 
                    "timesyncAccuracy_ms" : serviceData['timesyncAccuracy_ms'], 
                    "timesyncWaitService" : serviceData['timesyncWaitService'], 
                }
            ]
        )
    ])
