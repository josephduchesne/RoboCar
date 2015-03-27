#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"

#include <sstream>

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle node_handle;

  ros::Publisher chatter_pub = node_handle.advertise<std_msgs::Int16>("serial_node", 1); //queue size of 1

  ros::Rate loop_rate(20);

  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Int16 msg;

    msg.data = count;

    ROS_INFO("%d", msg.data);

    chatter_pub.publish(count);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
