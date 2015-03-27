#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"

#include <boost/algorithm/string.hpp>
#include <sstream>
#include <tr1/unordered_map>
#include <boost/asio.hpp>
using namespace::boost::asio;  // save tons of typing

std::tr1::unordered_map<std::string, ros::Publisher> pub;

serial_port_base::baud_rate serial_baud(115200);
io_service io;
serial_port serial(io, "/dev/ttyUSB0");


void parseSensorData( std::string line ){
    if (line.size() < 2) {
        return;
    }
    //ROS_INFO("Got serial: %s", line.c_str());

    int value;
    sscanf(line.c_str()+1, "%d", &value);

    switch (line[0]) {
      case 'D': //rangefinder
        {
          std_msgs::Float32 msg;
          msg.data = (float)value/100.0f; //convert from cm to m
          ROS_INFO("Distance: %fm", msg.data);
          pub["range"].publish(msg);
          break;
        }
      case 'L': //left wheel encoder
        {
          std_msgs::Int16 msg;
          msg.data = value;
          pub["left_encoder"].publish(msg);
          break;
        }
      case 'R': //right wheel encoder
        {
          std_msgs::Int16 msg;
          msg.data = value;
          pub["right_encoder"].publish(msg);
          break;
        }
      case 'P': //left wheel encoder
        {
          std_msgs::Int16 msg;
          msg.data = value;
          pub["pitch_angle"].publish(msg);
          break;
        }
      case 'Y': //right wheel encoder
        {
          std_msgs::Int16 msg;
          msg.data = value;
          pub["yaw_angle"].publish(msg);
          break;
        }
      case '#': //comment
        {
          break;
        }
      default:
        {
          //ROS_INFO("Unknown command %s", line.c_str());
          break;
        }
    }
}


boost::asio::streambuf serial_buffer;
void serial_read_handler(const boost::system::error_code& error,std::size_t bytes_transferred){
  //ROS_INFO("Read handler");

  if (!error) {
    std::istream is(&serial_buffer);
    std::string line;
    std::getline(is, line);
    //ROS_INFO("Got serial: %s", line.c_str());
    parseSensorData(line);
  }  else {
    ROS_INFO("Error!");
  }

  //request the next line
  if (ros::ok()) {
    ros::spinOnce();
    async_read_until(serial, serial_buffer, "\r\n", serial_read_handler); 
  } else {
    io.reset();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_node");

  ros::NodeHandle node_handle;

  //queue size of 101
  pub["left_encoder"] = node_handle.advertise<std_msgs::Int16>("left_wheel/encoder", 10);
  pub["right_encoder"] = node_handle.advertise<std_msgs::Int16>("right_wheel/encoder", 10);
  pub["pitch_angle"] = node_handle.advertise<std_msgs::Int16>("sensors/pitch_angle", 10);
  pub["yaw_angle"] = node_handle.advertise<std_msgs::Int16>("sensors/yaw_angle", 10);
  pub["range"] = node_handle.advertise<std_msgs::Float32>("sensors/range", 10);

  serial.set_option( serial_baud );

  ros::Duration(3).sleep(); //wait 3sec for the serial port to init

  async_read_until(serial, serial_buffer, "\r\n", serial_read_handler);
  io.run();

  return 0;
}
