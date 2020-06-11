from singleton import Singleton
import rospy
from std_msgs.msg import ColorRGBA, String, Float64, Empty #, Bool, UInt16, Empty
from sensor_msgs.msg import JointState #, Imu
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
from std_srvs.srv import Empty as SrvEmpty
from std_srvs.srv import EmptyResponse
from social_robot_arm_sdk.msg import JointPose
from social_msgs.srv import SocialMotion, SocialMotionResponse
from robocare_msgs.srv import TTSMake, SoundPlay, SoundPlayResponse
import time

class Service(Singleton):

    def __init__(self):
        
        __service = None

        __cmd_pos_publisher = None

        __earR_led_publisher = None
        __earL_led_publisher = None
        __torso_led_publisher = None
        __lidar_led_publisher = None
        __head_led_publisher = None
        __wheel_led_publisher = None

        __cur_Joint_subscriber = None


        __social_avatar_publisher = None

        __ctrl_module_publisher = None
        __play_motion_publisher = None

        __make_tts_service = None
        __play_sound_publisher = None
        __play_sound_stop_publisher = None

        self.__tts_count = -1
        self.__tts_file_path = '/tmp/robocare_tts'+str(self.__tts_count)+'.wav'
        self.connector()

    def connector(self):

        # TODO trigger
        rospy.Service('/social_motion_player/play_motion', SocialMotion, self.play_motion_handler)
        rospy.Service('/social_motion_player/stop', SrvEmpty, self.stop_handler)

        # TODO clee
        # self.__cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.__cmd_pos_publisher = rospy.Publisher('/cmd_pos', JointState, queue_size=10)
        # self.__reset_publisher = rospy.Publisher('/reset', Empty, queue_size=10)
        # self.__lidar_OnOff_publisher = rospy.Publisher('/lidar_OnOff', Bool, queue_size=10)
        # self.__waist_brake_publisher = rospy.Publisher('/waist_brake', Bool, queue_size=10)

        self.__earR_led_publisher = rospy.Publisher('/earR_led', ColorRGBA, queue_size=10)
        self.__earL_led_publisher = rospy.Publisher('/earL_led', ColorRGBA, queue_size=10)
        self.__torso_led_publisher = rospy.Publisher('/torso_led', ColorRGBA, queue_size=10)
        self.__lidar_led_publisher = rospy.Publisher('/lidar_led', ColorRGBA, queue_size=10)
        self.__head_led_publisher = rospy.Publisher('/head_led', ColorRGBA, queue_size=10)
        self.__wheel_led_publisher = rospy.Publisher('/wheel_led', ColorRGBA, queue_size=10)

        # self.__imu_subscriber = rospy.Subscriber('/imu', Imu, self.imu_handler)
        # self.__odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_handler)
        self.__cur_Joint_subscriber = rospy.Subscriber('/cur_Joint', JointState, self.cur_Joint_handler)
        # self.__Battery_subscriber = rospy.Subscriber('/Battery', UInt16, self.battery_handler)
        # self.__Version_subscriber = rospy.Subscriber('/Version', String, self.version_handler)

        self.__social_avatar_publisher = rospy.Publisher('/social_avatar/face', String, queue_size=10)

        # TODO arm
        self.__ctrl_module_publisher = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size=10)
        self.__play_motion_publisher = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=10)

        self.__joint_publishers = {}
        joint_list = [
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
                        'RFinger',
                        'Waist_Roll',
                        'Waist_Pitch',
                        'Head_Yaw',
                        'Head_Pitch'
                        ]
        for joint_name in joint_list:
            pub = rospy.Publisher('/social_robot/'+joint_name+'_position/command', Float64, queue_size=10)
            self.__joint_publishers[joint_name] = pub


        self.__make_tts_service = rospy.ServiceProxy('/robocare_tts/make', TTSMake)
        self.__play_sound_service = rospy.ServiceProxy('/robocare_sound/play', SoundPlay)
        self.__play_sound_stop_publisher = rospy.Publisher('/robocare_sound/stop', Empty, queue_size=10)

    def request_enable_ctrl_module(self):
        # print "arm_control_module"
        msg = String()
        msg.data = "none"
        # msg.data = "arm_motion_control_module"
        return self.__ctrl_module_publisher.publish(msg)

    def play_motion_handler(self, srv):
        result = self.__service.execute(srv)
        if result:
            return SocialMotionResponse.RESULT_SUCCESS
        else:
            return SocialMotionResponse.RESULT_FAILED

    def request_tts(self, text):
        self.__tts_count = self.__tts_count + 1
        if self.__tts_count > 10:
            self.__tts_count = 0
        self.__make_tts_service(text, self.__tts_file_path)

        self.__play_sound_service(self.__tts_file_path)

    def stop_handler(self, srv):
        self.__service.stop()
        return EmptyResponse()


    def request_social_avatar(self, msg):
        return self.__social_avatar_publisher.publish(msg)

    def request_cmd_pos(self, msg):
        for idx, name in enumerate(msg.name):
            if self.__joint_publishers.has_key(name):
                pub = self.__joint_publishers[name]
                pub.publish(msg.position[idx])
        return self.__cmd_pos_publisher.publish(msg)


    def request_earR_led(self, msg):
        return self.__earR_led_publisher.publish(msg)

    
    def request_earL_led(self, msg):
        return self.__earL_led_publisher.publish(msg)

    
    def request_torso_led(self, msg):
        return self.__torso_led_publisher.publish(msg)


    def request_lidar_led(self, msg):
        return self.__lidar_led_publisher.publish(msg)


    def request_head_led(self, msg):
        return self.__head_led_publisher.publish(msg)


    def request_wheel_led(self, msg):
        return self.__wheel_led_publisher.publish(msg)
    

    def request_play_motion(self, msg):
        for idx, name in enumerate(msg.name):
            if self.__joint_publishers.has_key(name):
                pub = self.__joint_publishers[name]
                pub.publish(msg.position[idx])
        # return True
        return self.__play_motion_publisher.publish(msg)

    def set_joint(self, msg):
        self.__play_motion_publisher.publish(msg)

    def cur_Joint_handler(self, msg):
        pass


    def subscribe(self, service):

        self.__service = service


    def unsubscribe(self, service):

        self.__service = None

if __name__ == '__main__':
    rospy.init_node('social_motion_player_service_test')
    service = Service()
    time.sleep(2)
    msg = JointState()
    msg.name = [
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
    msg.position = [
        0, -1.221, 0, 0, 0, 0,
        0, 1.221, 0, 0, 0, 0
    ]
    service.set_joint(msg)