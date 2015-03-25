#!/usr/bin/env python

import rospy, pygame

from geometry_msgs.msg import Twist 

pygame.init();
pygame.joystick.init();
joystick = pygame.joystick.Joystick(0);
joystick.init();

def mover():
  pubUrl = 'twist'
  pub = rospy.Publisher(pubUrl, Twist, queue_size=10)
  rospy.init_node('joyctl')
  rate = rospy.Rate(30) # 1hz
  while not rospy.is_shutdown():
    twist = Twist();

    events = pygame.event.get();
    
    twist.linear.x = -2 * joystick.get_axis(1);
    twist.angular.z = -2* joystick.get_axis(0);

    pub.publish(twist)
    rate.sleep()

if __name__ == '__main__':
  try:
    mover()
  except rospy.ROSInterruptException:
    pass
