#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import JointState
import time
import rospkg

class SocialArmPosSaver():

    def __init__(self, name):
        rospy.init_node('social_arm_pos_saver')
        print 'SocialArmPosSaver : '+str(name)
        self.__name = name
        self.connect()

    def connect(self):
        self.__cur_Joint_subscriber = rospy.Subscriber('/robotis/present_joint_states', JointState, self.cur_Joint_handler)
        rospy.spin()

    def cur_Joint_handler(self, msg):
        result_dir = rospkg.RosPack().get_path('social_robot_arm_gui')+'/arm/'
        result_file = result_dir+self.__name+'.yaml'
        value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for idx, joint_name in enumerate(msg.name):
            if joint_name == "LShoulder_Pitch":
                value[0] = float("{0:.2f}".format(msg.position[idx]*180/3.141592))
            elif joint_name == "LShoulder_Roll":
                value[1] = float("{0:.2f}".format(msg.position[idx]*180/3.141592))
            elif joint_name == "LElbow_Pitch":
                value[2] = float("{0:.2f}".format(msg.position[idx]*180/3.141592))
            elif joint_name == "LElbow_Yaw":
                value[3] = float("{0:.2f}".format(msg.position[idx]*180/3.141592))
            elif joint_name == "LWrist_Pitch":
                value[4] = float("{0:.2f}".format(msg.position[idx]*180/3.141592))
            elif joint_name == "LFinger":
                value[5] = float("{0:.2f}".format(msg.position[idx]*180/3.141592))
            elif joint_name == "RShoulder_Pitch":
                value[6] = float("{0:.2f}".format(msg.position[idx]*180/3.141592))
            elif joint_name == "RShoulder_Roll":
                value[7] = float("{0:.2f}".format(msg.position[idx]*180/3.141592))
            elif joint_name == "RElbow_Pitch":
                value[8] = float("{0:.2f}".format(msg.position[idx]*180/3.141592))
            elif joint_name == "RElbow_Yaw":
                value[9] = float("{0:.2f}".format(msg.position[idx]*180/3.141592))
            elif joint_name == "RWrist_Pitch":
                value[10] = float("{0:.2f}".format(msg.position[idx]*180/3.141592))
            elif joint_name == "RFinger":
                value[11] = float("{0:.2f}".format(msg.position[idx]*180/3.141592))
        ss = "# time parameter\n\nmov_time : 1.0"
        ss = ss +  "\t# movement time\n\n";
        ss = ss +  "# target pose [deg]\n\n";
        ss = ss +  "tar_pose :\n";
        ss = ss +  "  LShoulder_Pitch  : " + str(value[0])+ "\n"
        ss = ss +  "  LShoulder_Roll   : " + str(value[1])+ "\n"
        ss = ss +  "  LElbow_Pitch     : " + str(value[2])+ "\n"
        ss = ss +  "  LElbow_Yaw       : " + str(value[3])+ "\n"
        ss = ss +  "  LWrist_Pitch     : " + str(value[4])+ "\n"
        ss = ss +  "  LFinger          : " + str(value[5])+ "\n"
        ss = ss +  "  RShoulder_Pitch  : " + str(value[6])+ "\n"
        ss = ss +  "  RShoulder_Roll   : " + str(value[7])+ "\n"
        ss = ss +  "  RElbow_Pitch     : " + str(value[8])+ "\n"
        ss = ss +  "  RElbow_Yaw       : " + str(value[9])+ "\n"
        ss = ss +  "  RWrist_Pitch     : " + str(value[10]) + "\n"
        ss = ss +  "  RFinger          : " + str(value[11]) + "\n"
        print 'file save : '+result_file
        f = open(result_file, 'w')
        f.write(ss)
        f.close()
        rospy.signal_shutdown(1)
        

if __name__ == '__main__':
    service = SocialArmPosSaver('tpa')