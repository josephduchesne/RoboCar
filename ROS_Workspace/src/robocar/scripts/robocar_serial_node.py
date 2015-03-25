#!/usr/bin/env python
import rospy
import serial
import time #a project manager's dream
import sys
from std_msgs.msg import Int16
from std_msgs.msg import Float32

serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.5, writeTimeout=0.1)
time.sleep(3) #nap while the arduino boots

#publishers and subscribers
pub = {}
sub = {}

#parse a single line of sensor data and publish it to a ROS topic
def parseSensorData( line ):
    
    try:
        data = int(line[1:])
    except ValueError:
        data = None

    print ":Command %s" % line
    
    cmd = line[0]
    #at times like this, it seems odd to me that python doesn't have a native switch statement
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

#a series of fairly silly little writer functions.
#This could probably be more cleaver,
#but it is unlikely to be more simple
def set_motor_enable(value):
  print ":Motor enable: %d" % value
  serial_port.write("M%d\n" % int(value))
  return
def write_left_motor(msg):
  print ":Right PWM: %d" % int(msg.data)
  serial_port.write("L%d\n" % int(msg.data))
  return
def write_right_motor(msg):
  print ":Left PWM: %d" % int(msg.data)
  serial_port.write("R%d\n" % int(msg.data))
  return

def serial_node():
    rospy.init_node('robocar_serial')
    rate = rospy.Rate(1000) # 1khz. This should be plenty fast and not max out CPU usage for no reason

    pub['left_encoder'] = rospy.Publisher('left_wheel/encoder', Int16, queue_size=10)
    pub['right_encoder'] = rospy.Publisher('right_wheel/encoder', Int16, queue_size=10)
    pub['pitch_angle'] = rospy.Publisher('sensors/pitch_angle', Int16, queue_size=10)
    pub['yaw_angle'] = rospy.Publisher('sensors/yaw_angle', Int16, queue_size=10)


    sub['left_pwm'] = rospy.Subscriber('left_wheel/motor_pwm', Float32, write_left_motor)
    sub['right_pwm'] = rospy.Subscriber('right_wheel/motor_pwm', Float32, write_right_motor)

    set_motor_enable(1); #turn on motors

    input_buffer = ''

    while not rospy.is_shutdown():

        #read all input into the buffer
        new_data =  serial_port.read(serial_port.inWaiting()).strip('\x00');
        #sys.stdout.write(new_data) #avoiding newlines or spaces here
        input_buffer = input_buffer + new_data; 

        if '\r\n' in input_buffer:
            lines = input_buffer.split('\n');
            input_buffer = lines.pop() #the last element is a partial line or empty
            lines = filter(None, map(str.strip, lines)) #get rid of whitespace, and empty entries
            map(parseSensorData, lines)         

        rate.sleep()

if __name__ == '__main__':
    try:
        serial_node()
    except rospy.ROSInterruptException:
        pass
