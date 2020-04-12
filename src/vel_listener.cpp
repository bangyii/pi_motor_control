#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "pigpiod_if2.h"
#include <signal.h>

#define SERVO_PIN 26
#define MOTOR_PIN 19
#define MOTOR_FOR_MAX 1540
#define MOTOR_REV_MAX 1460
#define MOTOR_NEUTRAL 1500

int gpio, motorIndex = 1, servoIndex = 1;
int pwm_motor[] = {MOTOR_REV_MAX, MOTOR_NEUTRAL, MOTOR_FOR_MAX};
int pwm_servo[] = {1100, 1500, 1900};

float map(float in, float inMin, float inMax, float outMin, float outMax){
	return (in-inMin)/(inMax-inMin) * (outMax - outMin) + outMin;
}

void velCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	float forward = msg-> linear.x;
	float angular = msg-> angular.z;
	ROS_INFO("Velocity command received");

	//Forward control
	if(forward > 0 && motorIndex < 2) motorIndex ++;
	else if(forward < 0 && motorIndex > 0) motorIndex--;

	set_servo_pulsewidth(gpio, MOTOR_PIN, pwm_motor[motorIndex]);

	//Reverse control
	if(angular > 0 && servoIndex < 2) servoIndex++;
	else if(angular == -1 && servoIndex > 0) servoIndex--;

	set_servo_pulsewidth(gpio, SERVO_PIN, pwm_servo[servoIndex]);
}

void sigintHandler(int signum){
	ROS_INFO("Shutting down node");
	//Shutdown servos
	set_servo_pulsewidth(gpio, MOTOR_PIN, 0);
	set_servo_pulsewidth(gpio, SERVO_PIN, 0);

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
	if (set_servo_pulsewidth(gpio, SERVO_PIN, 1500))
		ROS_INFO("Set servo failed");
	if (set_servo_pulsewidth(gpio, MOTOR_PIN, MOTOR_NEUTRAL))
		ROS_INFO("Ser motor failed");

	ros::Duration(3).sleep();
	ROS_INFO("Armed");

	ros::spin();
//	//Main loop to handle Twist messages
//	while (ros::ok()) {
//		// ROS_INFO("Stepping servo");
//		ros::Duration(0.1).sleep();
//		if (step > 1530)
//			dir = -1;
//		else if (step < 1460)
//			dir = 1;
//		step = step + 4 * dir;
//		//set_servo_pulsewidth(gpio, SERVO_PIN, step);
//
//		if (step == 1500)
//			continue; //Skip 1500
//		if (stepOld >= 1500 && step < 1500) { //Commanded reverse
//			set_servo_pulsewidth(gpio, MOTOR_PIN, step);
//			set_servo_pulsewidth(gpio, MOTOR_PIN, stepOld);
//			set_servo_pulsewidth(gpio, MOTOR_PIN, step);
//		}
//		stepOld = step;
//
//		set_servo_pulsewidth(gpio, MOTOR_PIN, step);
//	}

	return 0;
}
