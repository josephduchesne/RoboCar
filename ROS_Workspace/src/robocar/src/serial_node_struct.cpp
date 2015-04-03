#include "ros/ros.h"
#include <sstream>
#include <string>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserScan.h"
#include <stdint.h>

#include <boost/algorithm/string.hpp>
#include <tr1/unordered_map>
#include <boost/asio.hpp>
using namespace::boost::asio;  // save tons of typing

std::tr1::unordered_map<std::string, ros::Publisher> pub;
std::tr1::unordered_map<std::string, ros::Subscriber> sub;

serial_port *serial;
serial_port_base::baud_rate serial_baud(115200);
io_service io;

#define SENSOR_READINGS 15
struct sensor_packet {
  int16_t servo_pitch;
  int16_t servo_yaw;
  int16_t left_wheel_encoder;
  int16_t right_wheel_encoder;
  int16_t rangefinder_distance;
  int16_t sweep[SENSOR_READINGS*2];
  uint8_t end1;
  uint8_t end2;
} __attribute__((__packed__));

void parseSensorData( struct sensor_packet sensorPacket){
  std_msgs::Float32 range;
  std_msgs::Int16 left_encoder;
  std_msgs::Int16 right_encoder;
  sensor_msgs::JointState joint_state;
  sensor_msgs::LaserScan laser_scan;

  if(sensorPacket.end1 != 'E' && sensorPacket.end2 != 'D'){
    ROS_INFO("Invalid packet ending");
    return;
  }

  laser_scan.range_min = 0.1f; //http://kb.pulsedlight3d.com/support/solutions/articles/5000548616-lidar-lite-specifications
  laser_scan.range_max = 40.0f;
  laser_scan.angle_min = -0.001;
  laser_scan.angle_max = 0.001;
  laser_scan.angle_increment = 0.002;
  laser_scan.scan_time = 0.0;
  laser_scan.ranges.push_back((float)sensorPacket.rangefinder_distance/100.0f); //convert from cm to m
  laser_scan.ranges.push_back((float)sensorPacket.rangefinder_distance/100.0f); //convert from cm to m
  laser_scan.intensities.push_back((float)sensorPacket.rangefinder_distance/100.0f); //convert from cm to m
  laser_scan.intensities.push_back((float)sensorPacket.rangefinder_distance/100.0f); //convert from cm to m
  laser_scan.header.frame_id = "rangefinder";
  laser_scan.header.stamp = ros::Time::now();
  pub["laser"].publish(laser_scan);

  //range.data = (float)sensorPacket.rangefinder_distance/100.0f; //convert from cm to m
  //pub["range"].publish(range);

  left_encoder.data = sensorPacket.left_wheel_encoder;
  pub["left_encoder"].publish(left_encoder);

  right_encoder.data = sensorPacket.right_wheel_encoder;
  pub["right_encoder"].publish(right_encoder);

  joint_state.name.push_back("servo_yaw");
  joint_state.position.push_back((float)sensorPacket.servo_yaw/180.0f*3.141592f); //convert to radians
  
  joint_state.name.push_back("servo_pitch");
  joint_state.position.push_back((float)sensorPacket.servo_pitch/180.0f*3.141592f); //convert to radians

  pub["joint_state"].publish(joint_state);
}


boost::asio::streambuf serial_buffer;
void serial_read_handler(const boost::system::error_code& error,std::size_t bytes_transferred){
  //ROS_INFO("Read handler");

  if (!error) {
    const char* serial_bytes = boost::asio::buffer_cast<const char*>(serial_buffer.data());
    serial_buffer.consume(bytes_transferred);

    //72 byte packet + null byte that Arduino seems to like tacking on
    //also sanity checking for the packet ending
    if (bytes_transferred == 72 || bytes_transferred == 73) { 
      struct sensor_packet sensorPacket;
      memcpy(&sensorPacket, serial_bytes, sizeof(sensorPacket));
      parseSensorData(sensorPacket);
      //ROS_INFO("Valid serial length %ld bytes", bytes_transferred);
    } else {
      ROS_INFO("Invalid serial: %ld bytes", bytes_transferred);
    }

  }  else {
    ROS_INFO("Error!");
  }

  //request the next line
  if (ros::ok()) {
    async_read_until(*serial, serial_buffer, "ED", serial_read_handler); 
  } else {
    io.reset();
  }
}

void send_serial_message(char field, int value) {
  //if we were using more than one spinner thread this would have to lock a mutex
  char output[16];
  sprintf(output, "%c%d\n", field, value);

  //ROS_INFO("Writing serial: %s", output);
  boost::asio::write(*serial, boost::asio::buffer(output, strlen(output)));
}

void write_left_motor(std_msgs::Float32 message) {
  send_serial_message('L', (int)message.data);
}

void write_right_motor(std_msgs::Float32 message) {
  send_serial_message('R', (int)message.data);
}

void write_camera_pitch(std_msgs::Int16 message) {
  send_serial_message('P', message.data);
}

void write_camera_yaw(std_msgs::Int16 message) {
  send_serial_message('Y', message.data);
}

void write_mode(std_msgs::Int16 message) {
  send_serial_message('M', message.data); //set mode
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1); //1 thread for async ROS spinning

  std::string port;
  ros::param::param<std::string>("~port", port, "/dev/ttyUSB0");
  ROS_INFO("Launching new serial node on %s", port.c_str());
  serial = new serial_port(io, port);

  //queue size of 1
  pub["left_encoder"] = node_handle.advertise<std_msgs::Int16>("left_wheel/encoder", 1);
  pub["right_encoder"] = node_handle.advertise<std_msgs::Int16>("right_wheel/encoder", 1);
  pub["joint_state"] = node_handle.advertise<sensor_msgs::JointState>("base/joint_state", 1);
  pub["laser"] = node_handle.advertise<sensor_msgs::LaserScan>("sensors/laser", 1);
  
  sub["left_pwm"] = node_handle.subscribe("left_wheel/motor_pwm", 1, write_left_motor);
  sub["right_pwm"] = node_handle.subscribe("right_wheel/motor_pwm", 1, write_right_motor); //right?
  sub["pitch"] = node_handle.subscribe("sensors/pitch", 1, write_camera_pitch); 
  sub["yaw"] = node_handle.subscribe("sensors/yaw", 1, write_camera_yaw); 
  sub["mode"] = node_handle.subscribe("sensors/mode", 1, write_mode); 

  serial->set_option( serial_baud );

  ros::Duration(2).sleep(); //wait 3sec for the serial port to init
  ROS_INFO("Connected");

  send_serial_message('M', 1); //enable the motors

  ros::Duration(0.1).sleep(); //give motor enable a moment

  async_read_until(*serial, serial_buffer, "ED", serial_read_handler);
  
  spinner.start();
  io.run();

  return 0;
}
