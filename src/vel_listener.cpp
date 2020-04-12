#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "pigpiod_if2.h"

#define SERVO_PIN 26
#define MOTOR_PIN 19
#define MOTOR_FOR_MAX 1540
#define MOTOR_REV_MAX 1460
#define MOTOR_NEUTRAL 1500

int gpio;

void velCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	float forward = msg-> linear.x;
	float angular = msg-> angular.z;

	if(forward == 0.8)
		set_servo_pulsewidth(gpio, MOTOR_PIN, MOTOR_FOR_MAX);

	else set_servo_pulsewidth(gpio, MOTOR_PIN, MOTOR_REV_MAX);
}

int main(int argc, char **argv) {
	int step = 1500, dir = 1, stepOld = 1500, neautral = 1500;

	ros::init(argc, argv, "vel_listener");
	ros::Rate r(50);
	ros::NodeHandle nh;
	ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 10, velCallback);


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

	//Main loop to handle Twist messages
	while (ros::ok()) {
		ros::spin();
		// ROS_INFO("Stepping servo");
		ros::Duration(0.1).sleep();
		if (step > 1530)
			dir = -1;
		else if (step < 1460)
			dir = 1;
		step = step + 4 * dir;
		//set_servo_pulsewidth(gpio, SERVO_PIN, step);

		if (step == 1500)
			continue; //Skip 1500
		if (stepOld >= 1500 && step < 1500) { //Commanded reverse
			set_servo_pulsewidth(gpio, MOTOR_PIN, step);
			set_servo_pulsewidth(gpio, MOTOR_PIN, stepOld);
			set_servo_pulsewidth(gpio, MOTOR_PIN, step);
		}
		stepOld = step;

		set_servo_pulsewidth(gpio, MOTOR_PIN, step);
	}

	return 0;
}
