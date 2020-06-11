#!/usr/bin/env python
#-*- coding:utf-8 -*-

from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Float64
from social_robot_arm_sdk.msg import JointPose
from service import Service


class Player: # Task

    __service = None
    __flag = True

    def __init__(self, Service):
        self.__service = Service

    def play(self, motion):
        # print '*'*50
        # print 'Player motion : ' + str(motion)
        if motion.has_key('head_waist'):
            headWaistMsg = JointState()
            motion['head_waist'][0] = motion['head_waist'][0]
            motion['head_waist'][1] = -1 * motion['head_waist'][1]
            motion['head_waist'][2] = -1 * motion['head_waist'][2]
            motion['head_waist'][3] = -1 * motion['head_waist'][3]
            headWaistMsg.name.extend(['Waist_Roll',
                                    'Waist_Pitch',
                                    'Head_Pitch',
                                    'Head_Yaw'])
            headWaistMsg.position.extend(motion['head_waist'])
            headWaistMsg.velocity.extend(motion['velocity'])

            cmd_pos_result = self.__service.request_cmd_pos(headWaistMsg)
            # print cmd_pos_result
        if motion.has_key('arm'):
            armMsg = JointPose()
            armJointState = JointState()
            armJointState.position.extend(motion['arm'])
            armJointState.name.extend([
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
                                    ])
            armMsg.pose = armJointState
            armMsg.mov_time = motion['arm_time']*1000
            play_motion_result = self.__service.request_play_motion(armJointState)
            # print play_motion_result
        if motion.has_key('led'):
            # earRLedMsg = ColorRGBA()
            # earLLedMsg = ColorRGBA()
            # torsoLedMsg = ColorRGBA()
            # lidarLedMsg = ColorRGBA()
            # headLedMsg = ColorRGBA()
            # wheelLedMsg = ColorRGBA()
            led_list = []
            for led in motion['led']:
                msg = ColorRGBA()
                rgb = led.split(',')
                msg.r = float(rgb[0])
                msg.g = float(rgb[1])
                msg.b = float(rgb[2])
                led_list.append(msg)
            #     print 'rgb : '+str(msg.r)+', '+str(msg.g)+', '+str(msg.b)
            # print '***************************'
            # print 'earRLedMsg : '+str(led_list[0].r)+', '+str(led_list[0].g)+', '+str(led_list[0].b)
            earR_led_result = self.__service.request_earR_led(led_list[0])
            # print earR_led_result
            earL_led_result = self.__service.request_earL_led(led_list[1])
            # print earL_led_result
            torso_led_result = self.__service.request_torso_led(led_list[2])
            # print torso_led_result
            lidar_led_result = self.__service.request_lidar_led(led_list[3])
            # print lidar_led_result
            head_led_result = self.__service.request_head_led(led_list[4])
            # print head_led_result
            wheel_led_result = self.__service.request_wheel_led(led_list[5])
            # print wheel_led_result
        # print '*'*50
        if motion.has_key('avatar'):
            social_avatar_result = self.__service.request_social_avatar(motion['avatar'])
        # print social_avatar_result
        

        # if cmd_pos_result and play_motion_result and earR_led_result and \
        #     earL_led_result and torso_led_result and lidar_led_result and \
        #         head_led_result and wheel_led_result and social_avatar_result is not None:
        #     self.__flag = False
        # print 'play motion fin...'
        return True
