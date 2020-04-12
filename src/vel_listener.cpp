#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "pigpiod_if2.h"

#define SERVO_PIN 26
#define MOTOR_PIN 19

void velCallback(const geometry_msgs::Twist::ConstPtr& msg){

}
 

int main(int argc, char** argv){
 ros::init(argc, argv, "vel_listener");
 ros::NodeHandle nh;
 ros::Subscriber vel_sub = nh.subscribe ("cmd_vel", 10, velCallback);

 int step = 1500, dir = 1, gpio, stepOld = 1500;
 gpio = pigpio_start(NULL, NULL);
 if(gpio < 0)  ROS_INFO("Failed to start");
 else ROS_INFO("Success");

 //Set servo to neutral and arm ESC
 if(set_servo_pulsewidth(gpio, SERVO_PIN, 1500)) ROS_INFO("Set servo failed");
 if(set_servo_pulsewidth(gpio, MOTOR_PIN, 1500)) ROS_INFO("Ser motor failed");
 
 int neutral = 1500;
 ros::Duration(3).sleep();
 ROS_INFO("Armed"); 

// set_servo_pulsewidth(gpio, MOTOR_PIN, 1530);
// ROS_INFO("Forward");
// ros::Duration(2).sleep();
// set_servo_pulsewidth(gpio, MOTOR_PIN, 1200);
// ROS_INFO("Brake");
// ros::Duration(0.5).sleep();
// set_servo_pulsewidth(gpio, MOTOR_PIN, neutral);
// ROS_INFO("Neutral");
// ros::Duration(0.5).sleep();
// set_servo_pulsewidth(gpio, MOTOR_PIN, 1460);
// ROS_INFO("Reverse");
// ros::Duration(2).sleep();
// set_servo_pulsewidth(gpio, MOTOR_PIN, 1600);
// set_servo_pulsewidth(gpio, MOTOR_PIN, neutral);
// ROS_INFO("Neutral");
 	
 while(ros::ok()){
 // ROS_INFO("Stepping servo");
  ros::Duration(0.1).sleep();
  if(step > 1530) dir = -1;
  else if(step < 1460) dir = 1;
  step = step + 4*dir;
  //set_servo_pulsewidth(gpio, SERVO_PIN, step);
  
  if(step == 1500) continue; //Skip 1500
  if(stepOld >= 1500 && step < 1500){ //Commanded reverse
   set_servo_pulsewidth(gpio, MOTOR_PIN, step);
   set_servo_pulsewidth(gpio, MOTOR_PIN, stepOld);
   set_servo_pulsewidth(gpio, MOTOR_PIN, step);
  }
  stepOld = step;
 
  set_servo_pulsewidth(gpio, MOTOR_PIN, step);
 }
 return 0;
}
