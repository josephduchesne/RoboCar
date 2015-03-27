#!/usr/bin/env python

import rospy, pygame, sys, os

from geometry_msgs.msg import Twist 

#prevent anything from trying to use X11
os.environ["SDL_VIDEODRIVER"] = "dummy"

pygame.init();
pygame.joystick.init();
joystick = pygame.joystick.Joystick(0);
joystick.init();


def mover():
  pubUrl = 'twist'
  rospy.init_node('joyctl')
  pub = rospy.Publisher(pubUrl, Twist, queue_size=1)
  rate = rospy.Rate(30) # 1hz

  rospy.loginfo("joyctl start")

  while not rospy.is_shutdown():
    twist = Twist();

    events = pygame.event.get();
    
    twist.linear.x = -1 * joystick.get_axis(1);
    twist.angular.z = -1* joystick.get_axis(0);

    pub.publish(twist)
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
