#!/usr/bin/env python
# -*- coding: utf-8 -*-

from social_arm_torque_release import SocialArmTorqueRelease
from social_arm_pos_saver import SocialArmPosSaver
from social_motion_maker import SocialMotionMaker

import sys
def release():
    SocialArmTorqueRelease()

def save():
    argv = sys.argv
    name = 'default_name'
    if len(argv) >= 2:
        name = argv[1]
    SocialArmPosSaver(name)

def main():
    argv = sys.argv
    name = 'test'
    if len(argv) >= 2:
        name = argv[1]
    SocialMotionMaker(name)