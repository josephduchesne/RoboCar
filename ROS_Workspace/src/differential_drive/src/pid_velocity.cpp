/*
   pid_velocity - takes messages on wheel_vtarget 
      target velocities for the wheels and monitors wheel for feedback
      
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"

#include <sstream>
#include <string>
#include <iostream>

#include <boost/circular_buffer.hpp>
#include <tr1/unordered_map>

/**
 * Moving average class for O(1) mean calculation
 */
class MovingAverage {

private:
    boost::circular_buffer<float> *q;
    double sum;
public:
    MovingAverage(int n)  {
        sum=0;
        q = new boost::circular_buffer<float>(n);
    }
    ~MovingAverage() {
        delete q;
    }
    void push(float v) {
        if (q->size() == q->capacity()) {
            double t=(double)q->front();
            sum-=t;
            q->pop_front();
        }
        q->push_back(v);
        sum+=(double)v;
    }
    double size() {
        return q->size();
    }
    double mean() {
        return sum/size();
    }
}; 

/**
 * PidVelocity class that takes encoder ticks and a desired speed
 */
class PidVelocity {
private:
    float target, motor, vel, integral, error, derivative, previous_error;
    float vel_threshold;
    int prev_encoder, wheel_prev, wheel_latest, wheel_mult;
    ros::Time then;
    ros::Time prev_pid_time;
    
    float Kp, Ki, Kd;
    int out_min, out_max, rate, ticks_per_meter, timeout_ticks, rolling_pts, ticks_meter;
    int  encoder_min, encoder_max, encoder_low_wrap, encoder_high_wrap, ticks_since_target;
    MovingAverage *prev_vel;  //empty one for now

    std::tr1::unordered_map<std::string, ros::Publisher> pub;

public:

    void wheelCallback(std_msgs::Int16 message) {
      int enc = (int)message.data;
      if (enc < encoder_low_wrap and prev_encoder > encoder_high_wrap) {
        wheel_mult = wheel_mult + 1;
      }
            
      if (enc > encoder_high_wrap and prev_encoder < encoder_low_wrap) {
        wheel_mult = wheel_mult - 1;
      }
           
         
      wheel_latest = (float)(enc + wheel_mult * (encoder_max - encoder_min)) / ticks_per_meter;
      prev_encoder = enc;
        
      ROS_INFO("Got wheel callback data:%d wheel_latest: %d mult: %d", message.data, wheel_latest, wheel_mult);
    }

    void targetCallback(std_msgs::Float32 message) {
      target = message.data;
      ticks_since_target = 0;
      ROS_INFO("Got target callback %f", message.data);
    }

    PidVelocity(int argc, char **argv) {
      ros::init(argc, argv, "pid_velocity");
      ros::NodeHandle node_handle;
      ROS_INFO("Started");
 
      //init paramaters
      target = motor = vel = integral = error = derivative = previous_error = 0;
      wheel_prev = wheel_latest = wheel_mult = 0;
      prev_encoder = 0;

      ros::param::param<float>("~Kp", Kp, 10.0f);
      ros::param::param<float>("~Ki", Ki, 10.0f);
      ros::param::param<float>("~Kd", Kd, 0.001f);

      ros::param::param<int>("~out_min", out_min, -255);
      ros::param::param<int>("~out_max", out_max, 255);
      ros::param::param<int>("~rate", rate, 30);
      ros::param::param<int>("~rolling_pts", rolling_pts, 2);
      ros::param::param<int>("~timeout_ticks", timeout_ticks, 4);
      ros::param::param<int>("ticks_meter", ticks_meter, 20);

      ros::param::param<float>("~vel_threshold", vel_threshold, 0.001f);

      ros::param::param<int>("encoder_min", encoder_min, -32768);
      ros::param::param<int>("encoder_max", encoder_max, 32768);

      ros::param::param<int>("wheel_low_wrap", encoder_low_wrap, (int)( 0.3f*(encoder_max-encoder_min) ) + (encoder_min));
      ros::param::param<int>("wheel_high_wrap", encoder_high_wrap, (int)( 0.7f*(encoder_max-encoder_min) ) + (encoder_min));

      prev_vel = new MovingAverage(rolling_pts);
      wheel_latest = 0;
      then = ros::Time::now();
      prev_pid_time= ros::Time::now();
      ROS_INFO("got Kp:%0.3f Ki:%0.3f Kd:%0.3f tpm:%d wrap: %d,%d", Kp, Ki, Kd, ticks_per_meter, encoder_low_wrap, encoder_high_wrap);

      //subscribers/publishers
      node_handle.subscribe("wheel", 10, &PidVelocity::wheelCallback, this);
      node_handle.subscribe("wheel_vtarget", 10, &PidVelocity::targetCallback, this);
      pub["motor"] = node_handle.advertise<std_msgs::Float32>("motor_cmd", 10);
      pub["vel"] = node_handle.advertise<std_msgs::Float32>("wheel_vel", 10);
    }

    void doPid() {
        ros::Duration pid_dt_duration = ros::Time::now() - prev_pid_time;
        float pid_dt = pid_dt_duration.toSec();
        prev_pid_time = ros::Time::now();
        
        error = target - vel;
        integral = integral + (error * pid_dt);
        // ROS_INFO("i = i + (e * dt):  %0.3f = %0.3f + (%0.3f * %0.3f)", integral, integral, error, pid_dt);
        derivative = (error - previous_error) / pid_dt;
        previous_error = error;
    
        motor = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
        if (motor > out_max) {
            motor = out_max;
            integral = integral - (error * pid_dt);
        }
        if (motor < out_min) {
            motor = out_min;
            integral = integral - (error * pid_dt);
        }
      
        if (target == 0) {
            motor = 0;
        }
    
        ROS_INFO("vel:%0.2f tar:%0.2f err:%0.2f int:%0.2f der:%0.2f ## motor:%0.2f ", vel, target, error, integral, derivative, motor);
    } 

    void appendVel(double new_vel) {
      prev_vel->push((float)new_vel);
    }

    void calcRollingVel() {
      vel = prev_vel->mean();
    }

    void calcVelocity() {
      ros::Duration dt_duration = ros::Time::now() - then;
      double dt = dt_duration.toSec();
      double cur_vel;
      ROS_INFO("caclVelocity dt=%0.3lf wheel_latest=%d wheel_prev=%d" , dt, wheel_latest, wheel_prev);

      //we haven't recieved an updated wheel lately
      if (wheel_latest == wheel_prev) {
        //estimate speed using time since last update
        cur_vel = 1.0/(double)ticks_per_meter / dt;
        if (abs(cur_vel) < vel_threshold) { //too slow, consider velocity 0
          ROS_INFO("Too slow current_vel: %lf, vel = 0", cur_vel);
          appendVel(0);
          calcRollingVel();
        } else {  //we're going a decent speed
          ROS_INFO("Fast enough current_vel: %lf", cur_vel);

          if (abs(cur_vel) < vel) {  //we're mo longer going vel
            ROS_INFO("Slowing down %lf", cur_vel); 
            appendVel(cur_vel);
            calcRollingVel();
          }
        }
      } else {
        //we've recieved a new wheel value
        cur_vel = (wheel_latest - wheel_prev) / dt;
        appendVel(cur_vel);
        calcRollingVel();
        ROS_INFO("wheel updated vel=%f", vel);
        wheel_prev = wheel_latest;
        then = ros::Time::now();
      }

      //publish the updated velocity
      std_msgs::Float32 msg;
      msg.data = vel;
      pub["vel"].publish(msg);
    }

    //this function is actually a misnomer
    //it spins repeatedly until the PID times out
    void spinOnce(ros::Rate r) {
      //flush the old values
      previous_error = integral = error = derivative = 0.0;
      delete prev_vel; //delete the old object stored at this pointer
      prev_vel = new MovingAverage(rolling_pts); //replace with a new one

      std_msgs::Float32 msg;

      //run until the PID times out, or the program quits
      while (ros::ok() && ticks_since_target < timeout_ticks) {
        calcVelocity();
        doPid();
        msg.data = motor;
        pub["motor"].publish(msg);

        ros::spinOnce(); //run any callbacks that are waiting in the wings
        r.sleep();

        ticks_since_target += 1;
        if (ticks_since_target == timeout_ticks) { //timeout, write 0 to motor
          msg.data = 0;
          pub["motor"].publish(msg);
        }
      }
    }

    void spin() {
      ros::Rate r(rate);
      then = ros::Time::now();
      ticks_since_target = timeout_ticks;
      wheel_prev = wheel_latest;

      while (ros::ok()) {
        spinOnce(r);
        ROS_INFO("SPINNING");
        r.sleep();
      }
    }

};

int main(int argc, char **argv)
{
  PidVelocity pidVelocity = PidVelocity(argc, argv);

  pidVelocity.spin();

  return 0;
}
