#!/usr/bin/env python
#-*- coding:utf-8 -*-

from service import Service
from loader import Loader
from social_motion_player.player import Player
import datetime
import rospy
import time
import thread

RATE = 200
class Executor:

    # __is_running = False
    __is_interrupt = False

    def __init__(self):

        print "EXECUTOR __init__"
        self.__if_fin_tts = True
        self.rate = rospy.Rate(RATE)
        self.__loader = Loader()
        self.__serivce = Service()

        self.__serivce.subscribe(self)
        time.sleep(0.5)
        self.enable_ctrl_module()

    def enable_ctrl_module(self):
        self.__serivce.request_enable_ctrl_module()
    
    def tts_thread(self, text):
        self.__serivce.request_tts(text)
        self.__if_fin_tts = True

    def execute(self, srv):
        self.__serivce.request_enable_ctrl_module()
        name = srv.file_name
        motions = self.__loader.read_motion(name)
        text = srv.text
        self.__is_fin_tts = False
        thread.start_new_thread(self.tts_thread, (text, ))
        # self.__serivce.request_tts(text)
        __is_okay = True
        self.__is_interrupt = False
        try:
            # self.__is_running = True
            start = datetime.datetime.now()
            # print 'start : '+str(start)
            motion_idx = 0
            print 'start play motion'
            while not self.__is_interrupt:
                if motion_idx >= len(motions):
                    break
                motion = motions[motion_idx]
                timeline = motion['timeline']
                curr = datetime.datetime.now()
                # print 'curr : '+str(curr)
                # print 'timeline : '+str(timeline)
                # print 'curr - start : '+str(curr - start)
                if curr - start >= datetime.timedelta(seconds = timeline):
                    # print 'play!'
                    player = Player(self.__serivce)
                    result = player.play(motion)
                    if not result:
                        # print '=============='
                        break
                    motion_idx = motion_idx + 1
                self.rate.sleep()
        except Exception as e:
            print e
            # self.__is_running = False
        print 'finish play motion'
        # print 'execute __is_okay : ' +str(__is_okay)
        # print 'execute __is_interrupt : ' +str(self.__is_interrupt)
        return __is_okay

    def stop(self):
        self.__is_interrupt = True
        return 

if __name__ == '__main__':
    start = datetime.datetime.now()
    print start
    time.sleep(1.0)
    curr = datetime.datetime.now()
    print curr
    if curr - start>= datetime.timedelta(milliseconds=500):
        print '11111111111'