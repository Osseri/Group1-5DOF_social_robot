#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospkg
import json
import os
import math
from pprint import pprint

HEAD_WAIST_LENGTH = 4

class Loader:

    __get_path = None

    def __init__(self):
        rospackage = rospkg.RosPack()
        self.__get_path = rospackage.get_path('social_motion_res')

    
    def read_motion(self, name):

        _file = self.__get_path+"/motion/"+name+".json"

        if not os.path.isfile(_file):
            print 'non exist file : '+_file
            return False
        else:
            print 'load file : '+_file

        with open(_file) as file:
            data = json.load(file)
        frames = data['frames']
        last_arm_time = 0.0
        check_last_head_waist = False
        last_head_waist = []
        for idx, frame in enumerate(frames):
            # print '-'*30
            # print str(frame)
            # print '-'*30
            if not frame.has_key('timeline'):
                break
            timeline = frame['timeline']
            if idx == 0:
                if frame.has_key('head_waist'):
                    check_last_head_waist = True
                    frame['velocity'] = [0.0, 0.0, 0.0, 0.0]
                    last_head_waist = frame['head_waist']
                    last_head_waist.append(timeline)
                if frame.has_key('arm'):
                    last_arm_time = timeline
                    frame['arm_time'] = 0.0
            else:
                # if frame == frames[-1]:
                #     del frame['timeline']
                #     break
                if frame.has_key('arm'):
                    arm_time = timeline - last_arm_time
                    frame['arm_time'] = arm_time
                    last_arm_time = timeline
                
                if frame.has_key('head_waist'):
                    if not check_last_head_waist:
                        check_last_head_waist = True
                        frame['velocity'] = [0.0, 0.0, 0.0, 0.0]
                    else:
                        head_waist_time = timeline - last_head_waist[4]
                        velocity_list = list()
                        for index in xrange(0, HEAD_WAIST_LENGTH):
                            delta_angle = frame['head_waist'][index]-last_head_waist[index]
                            velocity_list.append(abs(delta_angle / head_waist_time))

                        frame['velocity'] = velocity_list
                    last_head_waist = frame['head_waist']
                    last_head_waist.append(timeline)
        print 'motion time : '+str(frames[len(frames)-1]['timeline'])
        # pprint(frames[1:])
        return frames

# TODO EXAMPLE
# for pose in xrange(0, LENGTH_OF_JOINT_ARRAY): # 0,1,2,3,4,5,6,7
#     # POSE
#     pose_list.append(math.radians(data['frames'][frame+1]["position"][pose]))
#     # VELOCITY
#     degree = data['frames'][frame+1]["position"][pose] - data['frames'][frame]["position"][pose]
#     op_time = data['frames'][frame+1]["operating_time"] - data['frames'][frame]["operating_time"]
#     if degree == 0:
#         vel_list.append(0.7)
#     else:
#         vel_list.append(abs(math.radians(degree) / op_time))
##############################################################

# l = Loader()
# l.read_motion('sMotion_1.json')
