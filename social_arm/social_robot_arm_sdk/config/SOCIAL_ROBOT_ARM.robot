[ control info ]
control_cycle = 10   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB0 | 1000000  | LShoulder_Pitch
#/dev/ttyUSB1 | 1000000  | RShoulder_Pitch

[ device info ]
# TYPE    | PORT NAME    | ID   | MODEL         | PROTOCOL | DEV NAME        | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 1    | XM540-W270    | 2.0      | LShoulder_Pitch | present_position
dynamixel | /dev/ttyUSB0 | 2    | XM540-W270    | 2.0      | LShoulder_Roll  | present_position
dynamixel | /dev/ttyUSB0 | 3    | XM-430-W350   | 2.0      | LElbow_Pitch    | present_position
dynamixel | /dev/ttyUSB0 | 4    | XM540-W270    | 2.0      | LElbow_Yaw      | present_position
dynamixel | /dev/ttyUSB0 | 5    | XM-430-W350   | 2.0      | LWrist_Pitch    | present_position
dynamixel | /dev/ttyUSB0 | 6    | XM-430-W350   | 2.0      | LFinger         | present_position
dynamixel | /dev/ttyUSB0 | 7    | XM540-W270    | 2.0      | RShoulder_Pitch | present_position
dynamixel | /dev/ttyUSB0 | 8    | XM540-W270    | 2.0      | RShoulder_Roll  | present_position
dynamixel | /dev/ttyUSB0 | 9    | XM-430-W350   | 2.0      | RElbow_Pitch    | present_position
dynamixel | /dev/ttyUSB0 | 10   | XM540-W270    | 2.0      | RElbow_Yaw      | present_position
dynamixel | /dev/ttyUSB0 | 11   | XM-430-W350   | 2.0      | RWrist_Pitch    | present_position
dynamixel | /dev/ttyUSB0 | 12   | XM-430-W350   | 2.0      | RFinger         | present_position
