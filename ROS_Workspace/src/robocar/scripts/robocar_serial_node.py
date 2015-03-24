#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import Int16

port = serial.Serial('/dev/ttyUSB0', 115200)

pub = {}
pub['left_encoder'] = rospy.Publisher('state/left_encoder', Int16, queue_size=10)
pub['right_encoder'] = rospy.Publisher('state/right_encoder', Int16, queue_size=10)
pub['pitch_angle'] = rospy.Publisher('state/pitch_angle', Int16, queue_size=10)
pub['yaw_angle'] = rospy.Publisher('state/yaw_angle', Int16, queue_size=10)

def parseSensorData( line ):
    
    try:
        data = int(line[1:])
    except ValueError:
        data = None
    
    cmd = line[0];
    if cmd == 'L': #left wheel enc
        #rospy.loginfo("Left wheel: %d" % data)
        pub['left_encoder'].publish(data)
    elif cmd == 'R': #right wheel enc
        #rospy.loginfo("Right wheel: %d" % data)
        pub['right_encoder'].publish(data)
    elif cmd == 'P': #pitch
        #rospy.loginfo("Pitch: %d" % data)
        pub['pitch_angle'].publish(data)
    elif cmd == 'Y': #yaw
        #rospy.loginfo("Yaw: %d" % data)
        pub['yaw_angle'].publish(data)
    elif cmd == '#': #comment
        #rospy.loginfo("Comment: " % line)
        pass
    else:
        rospy.logerr("Unknown command: %s" % line)

    return

def talker():
    rospy.init_node('robocar_serial')
    rate = rospy.Rate(1000) # 1khz

    input_buffer = ''

    while not rospy.is_shutdown():

        #read all input into the buffer
        input_buffer = input_buffer + port.read(port.inWaiting()).strip('\x00'); 
        if '\r\n' in input_buffer:
            lines = input_buffer.split('\n');
            input_buffer = lines.pop() #the last element is a partial line or empty
            lines = filter(None, map(str.strip, lines)) #get rid of whitespace, and empty entries
            map(parseSensorData, lines)         

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
