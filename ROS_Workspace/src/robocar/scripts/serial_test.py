#!/usr/bin/env python
import serial
import time #a project manager's dream
import sys

serial_port = serial.Serial('COM5', 19200, timeout=0.01,  stopbits = 1, writeTimeout=1)
time.sleep(3) #nap while the arduino boots

#parse a single line of sensor data and publish it to a ROS topic
def parseSensorData( line ):
    if len(line) < 2:
        return
    
    print "Reading line outbuff: %d  inbuffer:%d" %  (serial_port.outWaiting(), serial_port.inWaiting())

    try:
        data = int(line[1:])
    except ValueError:
        data = None

    print ":Command %s" % line.strip()
    
    return

#a series of fairly silly little writer functions.
#This could probably be more cleaver,
#but it is unlikely to be more simple
def set_motor_enable(value):
  print ":Motor enable: %d" % value
  serial_port.write("M%d\n" % int(value))
  serial_port.flush()
  return
def write_left_motor(msg):
  print ":Right PWM: %d" % int(0)
  serial_port.write("L%d\n" % int(0))
  return

def serial_node():

    set_motor_enable(1); #turn on motors

    while True:
        while True:
          parseSensorData(serial_port.readline())
          if serial_port.inWaiting()<10:
            break; #out of while loop

        write_left_motor(0);
        write_left_motor(0);

if __name__ == '__main__':
    serial_node()
