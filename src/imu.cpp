#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

int main(int argc, char **argv)
{

	/* Serial communication with Arduino */ 
	struct termios toptions;
	int fd, n;
	double roll, pitch, yaw;
	std::string portname;
	
	/* ROS */
	ros::init (argc,argv,"imu_node");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Pose2D>("imu",1);
		ros::Rate loop_rate(50); //Hz

	geometry_msgs::Pose2D pose;
	/* Arduino's portname enters as rosparam */
	nh.getParam("imu_node/serial_port",portname);
	
	/* open serial port */
	fd = open(portname.c_str(), O_RDWR | O_NOCTTY);

	/* wait for the Arduino to reboot */
	usleep(3500000);

	/* get current serial port settings */
	tcgetattr(fd, &toptions);
	/* set 115200 baud both ways */
	cfsetispeed(&toptions, B115200);
	cfsetospeed(&toptions, B115200);
	/* 8 bits, no parity, no stop bits */
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	/* Canonical mode */
	toptions.c_lflag |= ICANON;
	/* commit the serial port settings */
	tcsetattr(fd, TCSANOW, &toptions);

	while(ros::ok())
	{
		char buf[64]="temp text";
		write(fd, "I\n", 2);
		usleep(500);
		/* Receive string from Arduino */
		do
		{
			n = read(fd, buf, 32);
		}while (n < 10);
		/* insert terminating zero in the string */
		buf[n] = 0;		
		sscanf(buf, "I|%lf|%lf|%lf|*\r\n", &pose.theta, &pitch, &roll);
		ROS_INFO_STREAM("yaw: " << pose.theta);
		pub.publish(pose);
		ros::spinOnce();
		loop_rate.sleep();
	}	
}