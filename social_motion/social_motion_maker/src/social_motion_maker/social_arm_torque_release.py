#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from social_robot_arm_sdk.msg import SetJointTorque
import time

class SocialArmTorqueRelease():

    def __init__(self):
        rospy.init_node('social_arm_torque_release')
        __cmd_pos_publisher = None
        self.__torque_publisher = rospy.Publisher('/arm/set_joint_torque', SetJointTorque, queue_size=10)

        time.sleep(1)
        arm_joint_list = [
                        'LShoulder_Pitch', 
                        'LShoulder_Roll', 
                        'LElbow_Pitch', 
                        'LElbow_Yaw', 
                        'LWrist_Pitch', 
                        'LFinger', 
                        'RShoulder_Pitch', 
                        'RShoulder_Roll', 
                        'RElbow_Pitch', 
                        'RElbow_Yaw', 
                        'RWrist_Pitch',
                        'RFinger'
                        ]
        msg = SetJointTorque()
        msg.joint_name = arm_joint_list
        msg.torque = [False, False, False, False, False, False, False, False, False, False, False, False]
        # msg.data = "arm_motion_control_module"
        self.__torque_publisher.publish(msg)

if __name__ == '__main__':
    SocialArmTorqueRelease()
