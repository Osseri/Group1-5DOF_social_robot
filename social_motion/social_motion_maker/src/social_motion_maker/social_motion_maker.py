#!/usr/bin/python
#-*-coding:utf-8-*-

import sys
import os
import thread
import time
import rospy
import rospkg
import json
from bson import json_util
from collections import OrderedDict

class SocialMotionMaker():
    def __init__(self, name):
        print "SocialMotionMaker "+name
        self.__name = name
        self.__joint_data = []
        self.__avatar_data = {}
        self.__led_data = {}
        self.__result_dict = OrderedDict()
        self.__result_dict['name'] = self.__name
        self.__frame_list = []
        self.__result_dict['frames'] = self.__frame_list

        self.parse_joint(name+'.joint')
        self.parse_led(name+'.json')
        self.parse_avatar(name+'.json')
        self.to_json()


    def parse_joint(self, file_name):
        joint_dir = rospkg.RosPack().get_path('social_motion_res')+'/joint/'
        file_path = joint_dir+file_name
        if os.path.exists(file_path):
            print 'parse_joint : '+str(file_path)
            f = open(file_path, 'r')
            while True:
                line = f.readline().decode('utf8')
                if not line: 
                    break
                # print(line)
                line = line.replace('\n', '')
                self.__joint_data.append(line)
            f.close()

    def parse_avatar(self, file_name):
        # 1: 놀람 8: 딴청
        avatar_dir = rospkg.RosPack().get_path('social_motion_res')+'/avatar/'
        file_path = avatar_dir+file_name
        if os.path.exists(file_path):
            print 'parse_avatar : '+str(file_path)
            with open(file_path) as f:
                datas = json.load(f)
            # {
            #     "time": 1.00,
            #     "avatar": "1-end"
            # },
                time = 0
                for data in datas["avatar"]:
                    time = data["time"]
                    avatar = data["avatar"]

                    self.__avatar_data[time] = avatar

                self.__avatar_data['final_time'] = time
            # print self.__avatar_data        

    def parse_led(self, file_name):
        led_dir = rospkg.RosPack().get_path('social_motion_res')+'/led/'
        file_path = led_dir+file_name
        if os.path.exists(file_path):
            print 'parse_led : '+str(file_path)
            with open(file_path) as f:
                datas = json.load(f)
            # {
            #     "time": 0.5,
            #     "led": {
            #         "earR" : "1,1,1", "earL" : "1,1,1", "torso": "0,0,0", "lidar": "0,0,0", "head" : "0,0,0", "wheel": "0,0,0"
            #     }
            # },
                time = 0
                for data in datas["led"]:
                    time = data["time"]
                    led_list = []
                    led_list.append(data["led"]["earR"])
                    led_list.append(data["led"]["earL"])
                    led_list.append(data["led"]["torso"])
                    led_list.append(data["led"]["lidar"])
                    led_list.append(data["led"]["head"])
                    led_list.append(data["led"]["wheel"])

                    self.__led_data[time] = led_list

                self.__led_data['final_time'] = time
            # print self.__led_data
# "timeline":0,
#             "arm":[
#                 0,0,0,0,0,0,0,0,0,0,0,0
#             ],
#             "head_waist":[
#                 0,0,0,0
#             ],
#             "led":[
#                 "0,0,0", "0,0,0", "0,0,0", "0,0,0", "0,0,0", "0,0,0"
#             ],
#             "avatar": "0"


    def to_json(self):
        print '*****'*10
        result_dir = rospkg.RosPack().get_path('social_motion_res')+'/motion/'
        result_file = result_dir+self.__name+'.json'
        print 'result_file : '+str(result_file)
        count = 0
        final_joint_time = len(self.__joint_data) * 0.02
        if self.__led_data.has_key('final_time'):
            final_led_time = self.__led_data['final_time']
        else:
            final_led_time = 0.0
        if self.__avatar_data.has_key('final_time'):
            final_avatar_time = self.__avatar_data['final_time']
        else:
            final_avatar_time = 0.0
        final_times = [final_joint_time, final_led_time, final_avatar_time]
        final_times.sort()
        time = 0
        while time <= final_times[len(final_times)-1]:
            time = float("{0:.2f}".format(count * 0.01))
            frame = OrderedDict()
            if count % 2 == 0 and count/2 < len(self.__joint_data):
                one_joint_data = self.__joint_data[count/2]
                frame['timeline'] = time
                # frame['avatar'] = '0'
                # frame['led'] = ["0,0,0", "0,0,0", "0,0,0", "0,0,0", "0,0,0", "0,0,0"]
                joint_sp = one_joint_data.split(',')
                head_waist_list = []
                for ix in range(12, 16):
                    head_waist_list.append(float(joint_sp[ix]))
                frame['head_waist'] = head_waist_list
                arm_list = []
                for ix in range(0, 12):
                    arm_list.append(float(joint_sp[ix]))
                frame['arm'] = arm_list

            count = count + 1
            if self.__led_data.has_key(time):
                frame['timeline'] = time
                led_list = []
                for led_data in self.__led_data[time]:
                    led_list.append(led_data)
                frame['led'] = led_list

            if self.__avatar_data.has_key(time):
                frame['timeline'] = time
                avatar = self.__avatar_data[time]
                frame['avatar'] = avatar                

            if frame.has_key('timeline'):
                self.__frame_list.append(frame)
        js = json.dumps(self.__result_dict, sort_keys=True, indent=4, separators=(',',' : '))
        f = open(result_file, 'w')
        f.write(js)
        f.close()

if __name__ == '__main__':
    name = 'manse'
    smm = SocialMotionMaker(name)
