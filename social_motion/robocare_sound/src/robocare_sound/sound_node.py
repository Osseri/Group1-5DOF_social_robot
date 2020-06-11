#!/usr/bin/python

import rospy
from std_msgs.msg import String, Empty, UInt8
from robocare_msgs.srv import SoundPlay, SoundPlayResponse

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

import os
import signal
import threading

players = []
lock = threading.RLock()

class Player(threading.Thread):
    def __init__(self, filepath, _async=False):
        self.filepath = filepath
        self.flag = True
        with lock:
            players.append(self)
        if _async:
            threading.Thread.__init__(self)
            self.start()
    def run(self):
        self.play()
    def play(self):
        if not os.path.isfile(self.filepath):
            with lock:
                players.remove(self)
            return SoundPlayResponse.SOUND_RESULT_FILE_NOT_FOUND
        result = SoundPlayResponse.SOUND_RESULT_DONE
        rospy.loginfo('playfile: %s', self.filepath)
        self.pipeline = Gst.parse_launch('playbin uri=file://' + self.filepath)
        self.pipeline.set_state(Gst.State.PLAYING)
        bus = self.pipeline.get_bus()
        while True:
            if not self.flag:
                break
            msg = bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.ERROR | Gst.MessageType.EOS | Gst.MessageType.STATE_CHANGED)
            if not msg:
                continue
            t = msg.type
            if t == Gst.MessageType.ERROR:
                err, dbg = msg.parse_error()
                rospy.logerr('%s : %s', msg.src.get_name(), err)
                result = SoundPlayResponse.SOUND_RESULT_FAILED
                break
            elif t == Gst.MessageType.EOS:
                rospy.loginfo('EOS: %s', self.filepath)
                break
            elif t == Gst.MessageType.STATE_CHANGED:
                old_state, new_state, pending_state = msg.parse_state_changed()
                if (new_state == Gst.State.NULL):
                    break
        self.pipeline.set_state(Gst.State.NULL)
        rospy.loginfo('done: %s', self.filepath)
        with lock:
            players.remove(self)
        return result
    def stop(self):
        rospy.loginfo('stopfile: %s', self.filepath)
        self.flag = False
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)

def on_play(data):
    player = Player(data.data, _async=True)

def on_play_service(req):
    player = Player(req.filepath)
    return player.play()

def on_stop(ignore):
    with lock:
        for player in players:
            player.stop()

def exit(signum, frame):
    on_stop(None)

def init():
    Gst.init(None)
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

