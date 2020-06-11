#!/usr/bin/python

import rospy
from std_msgs.msg import String, Empty, UInt8
from robocare_msgs.srv import SoundPlay, SoundPlayResponse

import os
import signal
import thread

def on_play(data):
    if not os.path.isfile(data.data):
        return
    cmd = 'play '+data.data
    thread.start_new_thread(play_thread, (cmd, ))

def play_thread(cmd):
    os.system(cmd)

def on_play_service(req):
    if not os.path.isfile(req.filepath):
        return SoundPlayResponse.SOUND_RESULT_FILE_NOT_FOUND
    cmd = 'play '+req.filepath
    os.popen(cmd)
    return SoundPlayResponse.SOUND_RESULT_DONE

def on_stop(ignore):
    os.system("kill -9 $(ps aux | grep 'play'| grep -e 'wav' -e 'mp3' | grep -v 'player' | awk '{print $2}')")

def exit(signum, frame):
    on_stop(None)

def init():
    rospy.init_node('robocare_sound', anonymous=True)
    rospy.Subscriber('/robocare_sound/play', String, on_play)
    rospy.Subscriber('/robocare_sound/stop', Empty, on_stop)
    rospy.Service('/robocare_sound/play', SoundPlay, on_play_service)
    rospy.loginfo('robocare_sound ready')
    rospy.spin()

def main():
    signal.signal(signal.SIGINT, exit)
    try:
        init()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    signal.signal(signal.SIGINT, exit)
    try:
        init()
    except rospy.ROSInterruptException:
        pass

