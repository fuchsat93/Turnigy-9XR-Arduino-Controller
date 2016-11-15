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
	// ROS Initializations and setting a publisher object.

	ros::init(argc, argv, "segway_teleop");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/segway/cmd_vel", 1000);

	geometry_msgs::Twist msg;

	// msg will hold the information of cmd_vel

	// Set termios structures, and a structure for the old settings
	struct termios tio;
	struct termios stdio;
	struct termios old_stdio;
	int tty_fd;

	// Save old stdio settings
	unsigned char c='D';
	tcgetattr(STDOUT_FILENO,&old_stdio);

	// Copy current stdio to stdio structure.  
	memset(&stdio,0,sizeof(stdio));

	// Enable local echoing of input characters
	// Convert a received carriage return into a newline
	// Turn on output processing and change newline into carriage return + live feed pair
	// 
	stdio.c_lflag |= ECHO;
	stdio.c_iflag |= ICRNL;
	stdio.c_oflag |= (OPOST | ONLCR);
	
	// Will wait until VMIN characters can be read
	stdio.c_cc[VMIN]=1;
	stdio.c_cc[VTIME]=0;

	
	tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
	tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

	// Currently, Arduino will automatically connect to /dev/ttyACM0
	// Need to change this to automatically always use one name
	tty_fd=open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);
	tcgetattr(tty_fd,&tio);


	tio.c_cflag &= ~CSIZE; 
	tio.c_cflag|= CS8|CREAD|CLOCAL; //8 Data bits
	tio.c_cflag &= ~PARENB; //No Parity
	tio.c_cflag &= ~CSTOPB; // 1 stop bit
	tio.c_lflag=0;

	// Will wait until VMIN characters can be read
	tio.c_cc[VMIN]=1;
	tio.c_cc[VTIME]=0;

	cfsetospeed(&tio,B57600);            // 57600 baud
	cfsetispeed(&tio,B57600);            // 57600 baud

	// Set attributes now
	tcsetattr(tty_fd,TCSANOW,&tio);
	tcsetattr(tty_fd,TCSAFLUSH,&tio);

	char rxString[40];

	float ch1, ch2, ch3, ch4, ch5, ch6;

	int ind;

	char inchar='b';

	// Initializations are done, tell ROS no errors happened
	ROS_INFO_STREAM("INITIALIZED!");
	
	// Need to edit this to be based on while(ros(ok))
	ros::Rate rate(2);
	while(ros::ok())
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

							// ROS_INFO_STREAM("Sending random velocity command:" << " linear=" << msg.linear.x << " angular=" << msg.angular.z);
							// printf("CH1: %d; CH2: %d; CH3: %d; CH4: %d; CH5: %d; CH6: %d; \n", ch1, ch2, ch3, ch4, ch5, ch6);

							// Publish the message
							pub.publish(msg);	
						}
					}
				}



			read(STDIN_FILENO,&c,1);   
			rate.sleep();                   
	}

	// Close the connection and write back the old stdio.
	write(STDOUT_FILENO,"\n",1);
	close(tty_fd);
	tcsetattr(STDOUT_FILENO,TCSANOW,&old_stdio);

	return EXIT_SUCCESS;
}
