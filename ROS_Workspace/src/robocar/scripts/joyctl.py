#!/usr/bin/env python

import rospy, pygame, sys, os, numpy

from geometry_msgs.msg import Twist 
from std_msgs.msg import Int16

#prevent anything from trying to use X11
os.environ["SDL_VIDEODRIVER"] = "dummy"

pygame.init();
pygame.joystick.init();
joystick = pygame.joystick.Joystick(0);
joystick.init();


def mover():
  pubUrl = 'twist'
  rospy.init_node('joyctl')
  pub = {}
  pub['twist'] = rospy.Publisher(pubUrl, Twist, queue_size=1)
  pub['pitch'] = rospy.Publisher('sensors/pitch', Int16, queue_size=1)
  pub['yaw'] = rospy.Publisher('sensors/yaw', Int16, queue_size=1)
  pub['mode'] = rospy.Publisher('sensors/mode', Int16, queue_size=1)
  rate = rospy.Rate(20) # 1hz

  anglePerHz = 1;
  pitch = 0;
  yaw = 0;
  mode = 1;
  modeButton = False;

  rospy.loginfo("joyctl start")

  while not rospy.is_shutdown():
    twist = Twist();

    events = pygame.event.get();
    
    twist.linear.x = -1 * joystick.get_axis(1);
    twist.angular.z = -1* joystick.get_axis(0);

    buttons = joystick.get_numbuttons()

    #for i in range(buttons):
    #  if joystick.get_button(i):
    #    print "Pressed: %d\n" % i;

    if joystick.get_button(10):
      pitch += anglePerHz
    if joystick.get_button(12):
      pitch -= anglePerHz
    if joystick.get_button(11):
      yaw += anglePerHz
    if joystick.get_button(13):
      yaw -= anglePerHz

    pitch = numpy.clip(pitch, -10, 100);

    yaw = numpy.clip(yaw, -45, 45);

    #toggle
    if joystick.get_button(1) and not modeButton:
      mode = 4-mode; #toggle between 1 and 3
      pub['mode'].publish(mode);
      modeButton = True;
    if not joystick.get_button(1) and modeButton:
      modeButton = False;

    pub['twist'].publish(twist)
    pub['pitch'].publish(pitch)
    pub['yaw'].publish(yaw)
    rate.sleep()

  rospy.loginfo("RosPy shutdown triggered")

STDERR = sys.stderr
def excepthook(*args):
  rospy.loginfo("Caught %s" % args)

sys.excepthook = excepthook

if __name__ == '__main__':
  try:
    mover()
  except rospy.ROSInterruptException:
    pass


rospy.loginfo("Script naturally exiting?")
