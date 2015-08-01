#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>

void move(ros::Publisher mainPub, ros::Publisher landPub, ros::Publisher takeOffPub, 
		ros::Rate someRate, char direction);


int main(int argc, char **argv){
	ros::init(argc, argv, "testing_tum_simulator");
	ros::NodeHandle nh;

	ros::Publisher takeOffPub = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
	ros::Publisher landPub = nh.advertise<std_msgs::Empty>("/ardrone/land", 1000);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Rate rate(20);

	struct termios old_tio, new_tio;
	unsigned char c;

	tcgetattr(STDIN_FILENO, &old_tio);
	new_tio = old_tio;

	new_tio.c_lflag &=(~ICANON & ~ECHO);

	tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

	do{
		c=getchar();
		std::cout << c << std::endl;	
		move(pub, landPub, takeOffPub, rate, c);
	} while (c != 'q');

	tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

void move(ros::Publisher mainPub, ros::Publisher landPub, ros::Publisher takeOffPub, 
		ros::Rate someRate, char direction){
	geometry_msgs::Twist msg;
	std_msgs::Empty takeoffmsg;
	std_msgs::Empty landingmsg;

	if(direction == 'w'){
		ROS_INFO_STREAM("FORWARD X: 1.0"); 
		msg.linear.x = 1.0;
	} else if (direction == 'a'){
		ROS_INFO_STREAM("LEFT: Y: 1.0");
		msg.linear.y = 1.0;
	} else if (direction == 'd'){
		ROS_INFO_STREAM("RIGHT: Y: -1.0"); 
		msg.linear.y = -1.0;
	} else if (direction == 's'){
		ROS_INFO_STREAM("LEFT: X: -1.0");
		msg.linear.x = -1.0;
	}

	int i = 0;
	while(ros::ok()){		
		if(i < 3){
			takeOffPub.publish(takeoffmsg);
			ROS_INFO_STREAM("Published take off.");
		}
		mainPub.publish(msg);

		ROS_INFO_STREAM("Published message.");
		i++;
		if(i > 50){
			landPub.publish(landingmsg);
			ROS_INFO_STREAM("Published landing.");		
		}
		someRate.sleep();
		if(i > 50){
			break;
		}
	}
}
