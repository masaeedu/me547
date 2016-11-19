#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <algorithm>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdexcept>
#include <stdint.h>
#include <math.h>

#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

using namespace std;
using namespace boost;

class serialConnectA255
{
private:
	double currX;
	double currY;
	double currZ;
	double currXRot;
	double currYRot;
	double currZRot;

	double motor_1;
	double motor_2;
	double motor_3;
	double motor_4;
	double motor_5;

	double curTheta_1;
	double curTheta_2;
	double curTheta_3;
	double curTheta_4;
	double curTheta_5;

	std::string serial_port_; //Serial Port name
	int baud_rate_;           //Baud Rate for Serial Port
	int fd_;                  //Serial Port file descriptor
	int error_count_;

public:
	serialConnectA255();

    ~serialConnectA255();

	int openPort();

	int closePort();

	int initialize();

	int writeString(string str);

	int readString(string &str);

	void sleepms(int milliseconds);

	int getArmPositionRW();

	int getAngles();

	int goReady();

	int grip_open();
		
	int grip_close();

	double* forwKine(double desTh_1, double desTh_2, double desTh_3, double desTh_4, double desTh_5);

	double* invKine(double desX, double desY, double desZ, double alpha);

	int goInZ(double desZ);

	// Function to move the robot to a desired position in the real world
	int moveRobot(double x, double y, double z, double z_rot, double y_rot, double x_rot);

	// Function to move each joint to its desired angle
	int moveTheta(double desSpeed, double theta_1, double theta_2, double theta_3, double theta_4, double theta_5);
};
