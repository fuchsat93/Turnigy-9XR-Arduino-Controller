#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc,char** argv)
{
	// ROS STUFF

	ros::init(argc, argv, "segway_teleop");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/segway/cmd_vel", 1000);

	geometry_msgs::Twist msg;

	// ROS STUFF END


	struct termios tio;
	struct termios stdio;
	struct termios old_stdio;
	int tty_fd;

	unsigned char c='D';
	tcgetattr(STDOUT_FILENO,&old_stdio);

	memset(&stdio,0,sizeof(stdio));

	stdio.c_lflag |= ECHO;
	stdio.c_iflag |= ICRNL;
	stdio.c_oflag |= (OPOST | ONLCR);
	stdio.c_cc[VMIN]=1;
	stdio.c_cc[VTIME]=0;


	tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
	tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

	tty_fd=open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);
	tcgetattr(tty_fd,&tio);


	tio.c_cflag &= ~CSIZE;
	tio.c_cflag|= CS8|CREAD|CLOCAL;
	tio.c_cflag &= ~PARENB; //No Parity
	tio.c_cflag &= ~CSTOPB; // 1 stop bit
	tio.c_lflag=0;


	tio.c_cc[VMIN]=1;
	tio.c_cc[VTIME]=0;



	cfsetospeed(&tio,B57600);            // 57600 baud
	cfsetispeed(&tio,B57600);            // 57600 baud

	tcsetattr(tty_fd,TCSANOW,&tio);
	tcsetattr(tty_fd,TCSAFLUSH,&tio);

	char rxString[40];


	float ch1, ch2, ch3, ch4, ch5, ch6;

	int ind;

	char inchar='b';

	ROS_INFO_STREAM("INITIALIZED!");

	while (c!='q')
	{

		if (read(tty_fd,&inchar,1)>0){
					if (1) {
						rxString[0]=inchar;
						ind=1;

						while(inchar != '\n'){
							if(read(tty_fd,&inchar,1)>0){
								if (ind<40) {
									rxString[ind]=inchar;
									ind++;

								} else {
									inchar = '\n';
								}
							}

						}
						if (ind<40) {
							sscanf(rxString,"%f\t%f\t%f\t%f\t%f\t%f", &ch1, &ch2, &ch3, &ch4, &ch5, &ch6);

							msg.linear.x = (ch4 - 50) / 50;
							msg.angular.z = 0-(ch2 - 50) / 50;

							//ROS_INFO_STREAM("Sending random velocity command:" << " linear=" << msg.linear.x << " angular=" << msg.angular.z);

							// Publish the message
							pub.publish(msg);

							//printf("CH1: %d; CH2: %d; CH3: %d; CH4: %d; CH5: %d; CH6: %d; \n", ch1, ch2, ch3, ch4, ch5, ch6);
						}
					}
				}



			read(STDIN_FILENO,&c,1);                      
	}



	write(STDOUT_FILENO,"\n",1);
	close(tty_fd);
	tcsetattr(STDOUT_FILENO,TCSANOW,&old_stdio);

	return EXIT_SUCCESS;
}
