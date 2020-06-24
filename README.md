# Social Robot Manual - 5DOF

***social package is meta-package includes several packages to control the social robot platform.*** 

***This package includes a social motion, arm, avatar, recharge, navigation.*** 


> ## Install Guide

0. Install Library

   > sudo apt install ros-kinetic-qt-build
   >
   > sudo apt install ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control

1. Make workspace

   > ​	mkdir -p /social_ws/src
   >
   > ​	cd social_ws/src
   >
   > ​	catkin_init_workspace
   >
   > ​	cd ..
   >
   > ​	catkin_make

2. bashrc

   > ​	vi ~/.bashrc
   >
   > ​	source ~/social_ws/devel/setup.bash
   >
   > ​	source ~/.bashrc

3. Install 5DOF_social_robot package

   > cd social_ws/src
   >
   > git clone --recurse-submodules https://git.robocare.io/social/5DOF_social_robot.git
   >
   > git clone https://github.com/ros-drivers/urg_c.git
   >
   > git clone https://github.com/ros-perception/laser_proc.git
   >
   > cd .. 
   >
   > catkin_make



> ## User Guide

## 2. MOTION

### GAZEBO 

$ roslaunch social_robot_arm_sdk social_robot_arm_sdk_for_gazebo.launch

$ roslaunch social_robot_gazebo social_robot_gazebo.launch 

재생버튼을 클릭해야 실행

---

### GUI

$ rosrun social_robot_arm_gui social_robot_arm_gui

---

### MOTION MAKER


1. 로봇 팔 토크 release
	$ rosrun social_motion_maker social_arm_torque_release
2. 로봇 팔 조인트 save
	$ rosrun social_motion_maker social_arm_pose_saver [arm_name]
	ex) rosrun social_motion_maker social_arm_pose_saver test
3. 로봇 팔 모션 make
	$ rosrun social_motion_maker social_motion_maker [motion_name]
	ex) rosrun social_motion_maker social_motion_maker test 


* Arm 파일 생성

> test라는 파일을 저장 후 파일 경로를 따라가 보면 test.yaml 파일이 생성됨을 확인.

* joint 파일 생성


> 만들어진 [arm_name].yaml파일과 Motion Time을 이용하여 몇 초동안 팔을 움직일 건지 Arm list에 추가
>
> 만들어진 [Head Waist].yaml파일과 Motion Time을 이용하여 몇 초동안 팔을 움직일 건지 Head Waist list에 추가
>
> Motion Play 버튼을 눌러 Gazebo에서 로봇의 움직임을 확인 후
>
> [joint_Name].joint를 write한 후 Play and Save 버튼 클릭

* Motion 파일 생성


> [avata_name].json , [led_name].json, [joint_name].joint 이름을 같게 한 후
>
>  rosrun social_motion_maker social_motion_maker [motion_name]을 입력하면
>
> [motion_name].json 파일이 만들어짐

---

### MOTION PLAYER

 
$ roslaunch social_motion_player motion.launch

$ rosservice call /social_motion_player/play_motion 

"file_name: '[motion_name]'
text: '[text to speech]'
with_home: false"


---

| 로봇 모션 | 이름(명령어)          | 표현 설명                                  |
| :-------: | :-------------------- | :----------------------------------------- |
|     1     | polite_hello          | 공손하게 손 모아 인사하기                  |
|     2     | hello2                | 양손 흔들어 인사하기                       |
|     3     | hello                 | 한손 흔들어 인사하기                       |
|     4     | question              | 머리 긁적이기                              |
|     5     | direction             | 오른팔로 ‘저기요’ 하듯 한쪽 지목하기       |
|     6     | guid_left             | 양손으로 왼쪽방향을 가리키며 ‘이쪽으로’    |
|     7     | guid_right            | 양손으로 오른쪽 방향을 가리키며 ‘이쪽으로’ |
|     8     | manse                 | '만세~’ 두 팔을 들기                       |
|     9     | attention             | 차렷하기                                   |
|    10     | eye_rub2              | 양 손으로 눈 비비기                        |
|    11     | eye_rub               | 오른 손으로 눈 비비기                      |
|    12     | hoho2                 | 양 손으로 입을 가리고 호호호 웃기          |
|    13     | hoho2                 | 오른 손으로 입을 가리고 호호호 웃기        |
|    14     | fighting              | 오른 손으로 파이팅 하기                    |
|    15     | fighting2             | 양 손으로 파이팅 하기                      |
|    16     | butterfly             | 나비처럼 나는 모션하기                     |
|    17     | pang                  | 가슴 두드리며 답답해하기                   |
|    18     | yes                   | 고개 끄덕 끄덕                             |
|    19     | no                    | 고개 가로 젓기                             |
|    20     | hand_shake            | 악수하기                                   |
|    21     | stretch               | 기지개 펴기                                |
|    22     | arm_stretch           | 양팔 스트레칭하기                          |
|    23     | OTL                   | 머리에 손 가져다대며 좌절하기              |
|    24     | right_hand_ok         | 오른손으로 주먹인사하기                    |
|    25     | two_hand_shy          | 양손으로 얼굴 가리고 부끄러워하기          |
|    26     | two_hand_circle       | 양팔 머리 위로 들어 동그라미 모양 만들기   |
|    27     | left_hand_nack        | 머리 뒤로 기울이며 뒤통수 잡기             |
|    28     | confidence            | 양손 허리에 대고 좌우로 움직이기           |
|    29     | two_hand_stumble      | 손 모아 팔 위아래로 움직이며 울상짓기      |
|    30     | right_hand_hello      | 오른손으로 경례 자세 하기                  |
|    31     | walking_up            | 팔 굽혀 앞뒤로 흔들며 걸어가는 모양새 내기 |
|    32     | two_hand_stretch      | 양손 앞으로 뻗어 손사래 치기               |
|    33     | two_hand_up           | 양팔 머리 위로 올렸다 내리며 호응하기      |
|    34     | right_hand_wink       | 오른손 눈가에 가져갔다가 떼며 윙크하기     |
|    35     | right_hand_high_five  | 오른손으로 하이파이브하기                  |
|    36     | two_hand_high_five    | 양손으로 하이파이브 하기                   |
|    37     | two_hand_show         | 양팔 앞에서 옆으로 벌리며 짜잔 하기        |
|    38     | right_hand_basketball | 화난 표정으로 농구하기                     |
|    39     | two_hand_respond      | 어리둥절한 표정으로 손바닥 보이기          |
|    40     | korean_greeting       | 세배하기                                   |



## 3. Auto Charging & Navigation

***By using 2D Lidar sensor and RGBD sensor in combination, obstacle detection and avoidance is possible in three dimensions instead of plane.***

### SOCIAL RECHARGE


$ rosrun social_recharge social_recharge_node


or, 런치파일을 만들어 주행 파라미터 값을 조절


<launch>
        <node name="social_recharge_node" pkg="social_recharge" type="social_recharge_node" output="screen">
                <param name="tarket_distance" value="0.4"/>
                <param name="trans_threshold" value="0.03"/>
                <param name="angle_threshold" value="5.0"/>
                <param name="min_x" value="0.0"/>
                <param name="max_x" value="2.0"/>
                <param name="min_y" value="-1.0"/>
                <param name="max_y" value="1.0"/>
                <param name="leave_distance" value="0.7"/>
        </node>
</launch>     



1. leave charger
	$ rosservice call /social_recharge/leave_station "{}"
2. dock charger
	$ rosservice call /social_recharge/dock_on_station "{}"




---

### SOCIAL NAVIGATION


$ roslaunch robocare_navigation robocare_navigation.launch


