#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "pigpiod_if2.h"
#include <signal.h>

#define SERVO_PIN 26
#define MOTOR_PIN 19
#define MOTOR_FOR_MAX 1555
#define MOTOR_REV_MAX 1440
#define MOTOR_NEUTRAL 1500

int gpio, motorIndex = 1, servoIndex = 1;

float map(float in, float inMin, float inMax, float outMin, float outMax){
	return (in-inMin)/(inMax-inMin) * (outMax - outMin) + outMin;
}

void velCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	float forward = msg->linear.x;
	float angular = msg->angular.z;
	ROS_DEBUG("Velocity command received");

	if(forward > 0) forward = map(forward, 0, 0.8, MOTOR_NEUTRAL, MOTOR_FOR_MAX);
	else if(forward < 0) forward = map(forward, -0.5, 0, MOTOR_REV_MAX, MOTOR_NEUTRAL);
	else forward = 1500;

	if(angular < 0) angular = map(angular, -1, 0, 2000, 1500);
	else if(angular > 0) angular = map(angular, 0, 1, 1500, 1000);
	else angular = 1550;

//	//Forward control
//	if(forward > 0 && motorIndex < 2) motorIndex ++;
//	else if(forward < 0 && motorIndex > 0) motorIndex--;

	set_servo_pulsewidth(gpio, MOTOR_PIN, forward);

//	//Reverse control
//	if(angular > 0 && servoIndex < 2) servoIndex++;
//	else if(angular == -1 && servoIndex > 0) servoIndex--;

	set_servo_pulsewidth(gpio, SERVO_PIN, angular);
}

void sigintHandler(int signum){
	ROS_INFO("Shutting down node");
	//Shutdown servos
	set_servo_pulsewidth(gpio, MOTOR_PIN, MOTOR_NEUTRAL);
	set_servo_pulsewidth(gpio, SERVO_PIN, 1550);

	pigpio_stop(gpio);
	ros::shutdown();
}

int main(int argc, char **argv) {
	int step = 1500, dir = 1, stepOld = 1500, neautral = 1500;

	ros::init(argc, argv, "vel_listener");
	ros::NodeHandle nh;
	ros::Rate r(50);
	ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 10, velCallback);
	signal(SIGINT, sigintHandler);

	//Connect to gpio daemon
	gpio = pigpio_start(NULL, NULL);
	if (gpio < 0)
		ROS_INFO("Daemon connection failed");
	else
		ROS_INFO("Daemon connection successful");

	//Set servo to neutral and arm ESC
	if (set_servo_pulsewidth(gpio, SERVO_PIN, 1550))
		ROS_INFO("Set servo failed");
	if (set_servo_pulsewidth(gpio, MOTOR_PIN, MOTOR_NEUTRAL))
		ROS_INFO("Ser motor failed");

	ros::Duration(3).sleep();
	ROS_INFO("Armed");

	ros::spin();

	return 0;
}
