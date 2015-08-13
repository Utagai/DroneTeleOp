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
	} while (c != 'p');

	tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

void move(ros::Publisher mainPub, ros::Publisher landPub, ros::Publisher takeOffPub, 
		ros::Rate someRate, char direction){
	geometry_msgs::Twist msg;
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
	} else if (direction == 'k'){
		ROS_INFO_STREAM("UP: Z: 1.0");
		msg.linear.z = 1.0;
	} else if (direction == 'j'){
		ROS_INFO_STREAM("DOWN: Z: -1.0");
		msg.linear.z = -1.0;
	} else if (direction == 'h'){
		ROS_INFO_STREAM("STOP");
	} else if (direction == 'l'){
		ROS_INFO_STREAM("LAND");
		landPub.publish(landingmsg);
	} else if (direction == 't'){
		ROS_INFO_STREAM("TAKEOFF");
		takeOffPub.publish(landingmsg); //identical messages for each so we're using it again
	} else if (direction == 'q'){
		ROS_INFO_STREAM("TURN CLOCKWISE: -1.0");
		msg.angular.z = -1.0;
	} else if (direction == 'e'){
		ROS_INFO_STREAM("TURN COUNTER CLOCKWISE: 1.0");
		msg.angular.z = 1.0;
	}

	int i = 0;
	while(ros::ok()){		
		mainPub.publish(msg);

		ROS_INFO_STREAM("Published message.");
		i++;

		someRate.sleep();
		break;	
	}
}
