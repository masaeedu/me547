#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
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
#include <serialConnectA255.h>

#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

using namespace std;
using namespace boost;

serialConnectA255::serialConnectA255()
{
    //Initialize variables
    fd_ = -1;

    //Advertise publishable topics

    //Subscribe to a topic

    //Create an internal node handler
    ros::NodeHandle n_private("~");

    //Grab the default values from the parameter server
    n_private.param("serial_port", serial_port_, std::string("/dev/ttyUSB0"));
    // Baud Rate
    n_private.param("baud_rate", baud_rate_, 57600); // Baud Rate
}

serialConnectA255::~serialConnectA255()
{
    ROS_INFO("De-constructor Called");
    closePort();
}

//Open the Serial Port
int serialConnectA255::openPort()
{
    //Check if serial port is already open. If so, close it.
    closePort();

    ROS_INFO("Opening CRS A255 Controller port");

    //Open the port
    /*
	1. O_RDWR : Open for reading and writing
	2. O_SYNC : Write I/O operations on the file descriptor complete as defined by synchronised I/O file integrity completion.
	3. O_NONBLOCK : If O_NONBLOCK is set, then open() function will return without blocking for the device to be ready or available. If O_NONBLOCK is clear, then the open() function will block the calling thread until the device is ready or available before returning.
	4. O_NOCTTY : If set and path identifies a terminal device, open() will not cause the terminal device to become the controlling terminal for the process.
	5. S_IRUSR : read permission, owner
	6. S_IWUSR : write permission, owner
	*/
    fd_ = open(serial_port_.c_str(), O_RDWR | O_SYNC | O_NONBLOCK | O_NOCTTY, S_IRUSR | S_IWUSR);

    //Check if opening the port was successfull
    if (fd_ == -1)
    {
        switch (errno)
        {
        case EACCES:
            cerr << "You probably don't have premission to open the port for reading and writing." << endl;
            return -1;
            break;
        case ENOENT:
            cerr << "The requested port does not exist. Is the IMU connected? Was the port name misspelled?" << endl;
            return -1;
            break;
        }

        ROS_ERROR("serialConnectA255::Exception, Unable to open serial port [%s]. %s", serial_port_.c_str(), strerror(errno));
    }

    //Try to lock the port
    struct flock fl;
    fl.l_type = F_WRLCK;    //Exclusive write lock
    fl.l_whence = SEEK_SET; //lock the whole file
    fl.l_start = 0;         //Start of the blocking lock
    fl.l_len = 0;           //Length of the blocking lock
    fl.l_pid = getpid();    //Process ID of the process holding the blocking lock

    //Check if the lock was successfull
    if (fcntl(fd_, F_SETLK, &fl) != 0)
    {
        ROS_ERROR("serialConnectA255::Exception, Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", serial_port_.c_str(), serial_port_.c_str());
        return -1;
    }

    //Try to change port settings
    struct termios term;

    //Try and get port attributes
    if (tcgetattr(fd_, &term) < 0)
    {
        ROS_ERROR("serialConnectA255::Exception, Unable to get serial port attributes. The port you specified (%s) may not be a serial port.", serial_port_.c_str());
        return -1;
    }

    cfmakeraw(&term);           /*Sets the terminal to something like the "raw" mode of the old Version 7 terminal driver: input is available character by character, echoing is disabled, and all special processing of terminal input and output characters is disabled. The terminal attributes are set as follows:
					  termios_p->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
					  | INLCR | IGNCR | ICRNL | IXON);
					  termios_p->c_oflag &= ~OPOST;
					  termios_p->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
					  termios_p->c_cflag &= ~(CSIZE | PARENB);
					  termios_p->c_cflag |= CS8;
					  */
    cfsetispeed(&term, B57600); //Set input baud rate
    cfsetospeed(&term, B57600); //Set output baud rate

    //Set the parameters associated with the terminal
    if (tcsetattr(fd_, TCSAFLUSH, &term) < 0)
    {
        ROS_ERROR("serialConnectA255::Exception, Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", serial_port_.c_str()); /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.
        return -1;
    }

    // Make sure queues are empty before we begin
    if (tcflush(fd_, TCIOFLUSH) != 0)
    {
        cerr << "serialConnectA255::Exception, Tcflush failed. Please report this error if you see it." << endl;
        return -1;
    }

    return 0;
}

//Close the Serial Port
int serialConnectA255::closePort()
{
    ROS_INFO("Closing Port");
    //Check if the port is already open or not
    if (fd_ != -1)
    {
        // Exit ash test
        ROS_INFO("Exiting 'ash test'");
        int status;
        status = writeString("exit\r");
        if (status != 0)
        {
            cerr << "Unable to send 'exit'" << endl;
            return -1;
        }
        sleepms(100);
        status = writeString("y\r");

        if (status != 0)
        {
            cerr << "Unable to send 'y'" << endl;
            return -1;
        }
        sleepms(100);
        if (close(fd_) != 0)
        {
            ROS_ERROR("Unable to close serial port; [%s]", strerror(errno));
            return -1;
        }
        fd_ = -1;
        return 0;
    }
}

int serialConnectA255::initialize()
{
    try
    {
        string read;

        // Get into "ash test"

        int status;
        ROS_INFO("Entering 'ash test'");

        status = writeString("ash test\r");
        if (status != 0)
        {
            cerr << "Unable to enter 'ash test' mode." << endl;
            return -1;
        }

        sleepms(1000);
        // Drain command data from buffer
        status = readString(read);
        sleepms(100);
        status = readString(read);
        sleepms(100);

        // Get the version information for application shell
        ROS_INFO("Reading Robot Version information");
        status = writeString("robotver\r");
        sleepms(1000);

        // Try and read the string
        status = readString(read);
        if (status != 0)
        {
            cerr << "Unable to read 'robotver'" << endl;
            return -1;
        }
        read = read.substr(27, read.length() - 36);
        std::cout << read << std::endl;
        sleepms(1000);

        // Find Current Position
        getArmPositionRW();
        sleepms(1000);
        //go to Ready position
        goReady();

        return 0;
    }
    catch (std::exception &e)
    {
        error_count_++;
        ROS_ERROR("Caught an exception of an unexpected type: %s", e.what());
        return -1;
    }
    catch (...)
    {
        cerr << "Caught an unknown exception." << endl;
        return -1;
    }
}

int serialConnectA255::getArmPositionRW()
{
    ROS_INFO("Getting Robot Arm Current Position");

    // Flush read buffer
    int status;
    string read;
    status = readString(read);

    status = writeString("here\r");
    if (status != 0)
    {
        cerr << "Unable to send 'here'" << endl;
        return -1;
    }
    sleepms(100);

    status = readString(read);
    if (status != 0)
    {
        cerr << "Unable to read 'here'" << endl;
        return -1;
    }
    //std::cout << read << std::endl;
    //std::cout << "read length : " << read.length() << std::endl;
    read = read.substr(169, read.length());
    //std::cout << read << std::endl;

    //std::cout << read << std::endl;
    vector<string> fields;
    split(fields, read, is_any_of("\n"));
    read = fields[0];
    trim(read);
    //std::cout << read << std::endl;
    //Split again on the space
    split(fields, read, is_any_of(" "));

    int i = 0;
    //cout << "fields size: " << fields.size() << "\n";
    for (size_t n = 0; n < fields.size(); n++)
    {
        if (!iequals(fields[n], ""))
        {
            switch (i)
            {
            case 0:
                currX = lexical_cast<double>(fields[n]);
                //cout << "x : " << currX << "\n";
                i++;
                break;
            case 1:
                currY = lexical_cast<double>(fields[n]);
                //cout << "y : " << currY << "\n";
                i++;
                break;
            case 2:
                currZ = lexical_cast<double>(fields[n]);
                //cout << "z : " << currZ << "\n";
                i++;
                break;
            case 3:
                currZRot = lexical_cast<double>(fields[n]);
                //cout << "zRot : " << currZRot << "\n";
                i++;
                break;
            case 4:
                currYRot = lexical_cast<double>(fields[n]);
                //cout << "yRot : " << currYRot << "\n";
                i++;
                break;
            case 5:
                currXRot = lexical_cast<double>(fields[n]);
                //cout << "xRot : " << currXRot << "\n";
                i++;
                break;
            default:
                break;
            }
        }
        //cout << "\"" << fields[ n ] << "\"\n";
    }
    return 0;
}

string setPrecision(double input, int prec)
{
    stringstream buff;
    string temp;
    buff << setprecision(prec) << setiosflags(ios_base::fixed) << input; // decimal precision
    buff >> temp;
    return temp;
}

// Function to get the joint angles (in motor counts) and convert to angle values in CRS convention:
int serialConnectA255::getAngles()
{
    ROS_INFO("Reading Joint Angles");

    // Flush read buffer
    int status;
    string readAng;
    status = readString(readAng);

    status = writeString("w2\r");
    if (status != 0)
    {
        cerr << "Unable to send 'w2'" << endl;
        return -1;
    }
    sleepms(100);

    status = readString(readAng);
    if (status != 0)
    {
        cerr << "Unable to read 'w2'" << endl;
        return -1;
    }
    sleepms(200);

    //This line to be changed...
    readAng = readAng.substr(106, readAng.length());
    sleepms(200);

    vector<string> fieldsAng;
    split(fieldsAng, readAng, is_any_of("\n"));
    readAng = fieldsAng[0];
    trim(readAng);

    //Split again on the space
    split(fieldsAng, readAng, is_any_of(" "));
    cout << readAng << "\n";

    int i = 0;
    for (size_t n = 0; n < fieldsAng.size() - 1; n++)
    {
        if (!iequals(fieldsAng[n], ""))
        {
            switch (i)
            {
            case 0:
                motor_1 = lexical_cast<double>(fieldsAng[n]);
                curTheta_1 = motor_1 / 200.0;
                cout << "curTheta_1: " << curTheta_1 << "\n";
                sleepms(200);
                i++;
                break;
            case 1:
                motor_2 = lexical_cast<double>(fieldsAng[n]);
                curTheta_2 = -motor_2 / 200.0;
                cout << "curTheta_2: " << curTheta_2 << "\n";
                sleepms(200);
                i++;
                break;
            case 2:
                motor_3 = lexical_cast<double>(fieldsAng[n]);
                curTheta_3 = motor_3 / 200.0;
                cout << "curTheta_3: " << curTheta_3 << "\n";
                sleepms(200);
                i++;
                break;
            case 3:
                motor_4 = lexical_cast<double>(fieldsAng[n]);
                curTheta_4 = -motor_4 / 44.5;
                cout << "curTheta_4: " << curTheta_4 << "\n";
                sleepms(200);
                i++;
                break;
            case 4:
                motor_5 = lexical_cast<double>(fieldsAng[n]);
                curTheta_5 = (motor_4 + motor_5) / 22.2;
                cout << "curTheta_5: " << curTheta_5 << "\n";
                sleepms(200);
                i++;
                break;
            default:
                break;
            }
        }
    }

    return 0;
}

//Forward Kinematics
/*
double* serialConnectA255::forwKine(double desTh_1, double desTh_2, double desTh_3, double desTh_4, double desTh_5)
{
ROS_INFO("Solving Forward Kinematics");

double xFound;

//Convert degrees to radians
const double PI = 22/7;
desTh_1 = desTh_1*PI/180;
desTh_2 = desTh_2*PI/180;
desTh_3 = desTh_3*PI/180;
desTh_4 = desTh_4*PI/180;
desTh_5 = desTh_5*PI/180;
sleepms(500);

double *forwK=new double[3];

//Fill in the following:
forwK[0] = ;
forwK[1] = ;
forwK[2] = ;

//Print
cout << "xDes : " << forwK[0] << "\n";
cout << "yDes : " << forwK[1] << "\n";
cout << "zDes : " << forwK[2] << "\n";

return forwK;
}
*/

//Inverse Kinematics
//

//desX, desY, desZ are in millimeters, alpha in degrees
double *serialConnectA255::invKine(double desX, double desY, double desZ, double alpha)
{
    ROS_INFO("Solving Inverse Kinematics");

    //Declare Variables
    const double pi = 22.0 / 7;
    cout << "Pi Calculation, is IT a INTEGER?!? " << pi << "\n";
    alpha = alpha * pi / 180;

    double r = sqrt(desX * desX + desY * desY);
    double s = (desZ - 254); // subtract d_1 from the height of the end-effector
    double A2 = 254.0;
    double A3 = 254.0;
    double F = (pow(desX, 2) + pow(desY, 2) + pow(s, 2) - pow(A2, 2) - pow(A3, 2)) / (2 * A2 * A3); // cos(theta_3)
    cout << "F" << F << endl;

    //double F = 1;
    double theta_1 = atan2(desY, desX);                  // There is no offset "d".
    double theta_3 = atan2(-1 * sqrt(1 - pow(F, 2)), F); // theta_3 with respect to theLink 2 frame.

    cout << "theta3, " << atan2(-1 * sqrt(1 - pow(F, 2)), F) << endl;

    double theta_2 = atan2(s, r) - atan2(A3 * sin(theta_3), A2 + A3 * cos(theta_3));

    cout << "theta2 components, " << atan2(s, r) << ", " << atan2(A3 * sin(theta_3), A2 + A3 * cos(theta_3)) << endl;

    double theta_4 = -theta_3 - theta_2 - pi / 2;
    double theta_5 = alpha; //theta_1-alpha-pi/2 ;

    double *invK = new double[5];

    //Convert from DH to CRS
    invK[0] = theta_1 * 180 / pi;
    invK[1] = theta_2 * 180 / pi;
    invK[2] = (theta_3 + theta_2) * 180 / pi;
    invK[3] = (theta_4 + theta_3 + theta_2) * 180 / pi;
    invK[4] = (theta_1 + theta_5) * 180 / pi;

    //Print
    cout << "DesTh_1 : " << invK[0] << "\n";
    cout << "DesTh_2 : " << invK[1] << "\n";
    cout << "DesTh_3 : " << invK[2] << "\n";
    cout << "DesTh_4 : " << invK[3] << "\n";
    cout << "DesTh_5 : " << invK[4] << "\n";
    sleepms(500);

    return invK;
}
//

//moveTheta function
int serialConnectA255::moveTheta(double desSpeed, double finalTheta_1, double finalTheta_2, double finalTheta_3, double finalTheta_4, double finalTheta_5)
{
    // Get Current Arm Position
    getAngles();

    // Set the joint speed
    ostringstream spConverted;
    spConverted << desSpeed; // convert from int to str
    string tempSp = "";
    tempSp = "speed";
    tempSp = tempSp + " " + spConverted.str() + "\r";
    cout << tempSp << "\n";

    int status = writeString(tempSp);
    if (status != 0)
    {
        cerr << "Unable to send desired speed command" << endl;
        return -1;
    }
    sleepms(200);
    tempSp = "";

    // Calculate Offsets between current and desired angles
    double offsetTheta_1 = 0.0;
    double offsetTheta_2 = 0.0;
    double offsetTheta_3 = 0.0;
    double offsetTheta_4 = 0.0;
    double offsetTheta_5 = 0.0;

    offsetTheta_1 = finalTheta_1 - curTheta_1;
    offsetTheta_2 = finalTheta_2 - curTheta_2;
    offsetTheta_3 = finalTheta_3 - curTheta_3;
    offsetTheta_4 = finalTheta_4 - curTheta_4;
    offsetTheta_5 = finalTheta_5 - curTheta_5;

    string tempStr = "";
    int jointNo = 0;
    int kk = 1;
    for (kk; kk < 6; kk++)
    {
        jointNo = kk;
        ostringstream kkConverted;
        kkConverted << kk; // convert from int to str
        tempStr = "joint";
        ostringstream deg1Converted;
        ostringstream deg2Converted;
        ostringstream deg3Converted;
        ostringstream deg4Converted;
        ostringstream deg5Converted;
        switch (jointNo)
        {
        case 1:
            deg1Converted << offsetTheta_1; // convert from int to str
            tempStr = tempStr + " " + kkConverted.str() + "," + deg1Converted.str() + "\r";
            break;
        case 2:
            deg2Converted << offsetTheta_2; // convert from int to str
            tempStr = tempStr + " " + kkConverted.str() + "," + deg2Converted.str() + "\r";
            break;
        case 3:
            deg3Converted << offsetTheta_3; // convert from int to str
            tempStr = tempStr + " " + kkConverted.str() + "," + deg3Converted.str() + "\r";
            break;
        case 4:
            deg4Converted << offsetTheta_4; // convert from int to str
            tempStr = tempStr + " " + kkConverted.str() + "," + deg4Converted.str() + "\r";
            break;
        case 5:
            deg5Converted << offsetTheta_5; // convert from int to str
            tempStr = tempStr + " " + kkConverted.str() + "," + deg5Converted.str() + "\r";
            break;
        default:
            break;
        }
        // Send the joint command
        cout << tempStr << "\n";
        status = writeString(tempStr);
        if (status != 0)
        {
            cerr << "Unable to send joint command" << endl;
            return -1;
        }
        sleepms(3000);
        tempStr = "";
    }
    // Get Current Arm Position
    getAngles();
}

int serialConnectA255::moveRobot(double x, double y, double z, double z_rot, double y_rot, double x_rot)
{
    // Get Current Arm Position
    getArmPositionRW();
    // Calculate Offsets between current and desired
    double offsetX = 0.0;
    double offsetY = 0.0;
    double offsetZ = 0.0;
    double offsetZRot = 0.0;
    double offsetYRot = 0.0;
    double offsetXRot = 0.0;

    offsetX = x - currX;
    offsetY = y - currY;
    offsetZ = z - currZ;
    offsetZRot = z_rot - currZRot;
    offsetYRot = y_rot - currYRot;
    offsetXRot = x_rot - currXRot;

    // Declare a dummy variable called dummypoint
    int status = writeString("here dummypoint\r");
    if (status != 0)
    {
        cerr << "Unable to send 'here dummypoint'" << endl;
        return -1;
    }
    sleepms(100);

    // Send a wshift command with offsets
    string wShift;
    wShift = "wshift dummypoint,";
    wShift = wShift + setPrecision(offsetX, 3) + "," + setPrecision(offsetY, 3) + "," + setPrecision(offsetZ, 3) + "," + setPrecision(offsetZRot, 3) + "," + setPrecision(offsetYRot, 3) + "," + setPrecision(offsetXRot, 3) + "\r";
    std::cout << "shifting dummypoint by : " << wShift << "\n" ;
    status = writeString(wShift);
    if (status != 0)
    {
        cerr << "Unable to send wshift command" << endl;
        return -1;
    }
    sleepms(100);

    // Flush stdout
    string read;
    status = readString(read); 
    sleepms(100);

    // Send a move command
    status = writeString("moves dummypoint\r");
    if (status != 0)
    {
        cerr << "Unable to send move command" << endl;
        return -1;
    }
    sleepms(100);

    // Read command output_handle
    status = readString(read);
    if (status != 0)
    {
        cerr << "Unable to read move command output" << endl;
        return -1;
    }

    // Check for out-of-reach error message
    istringstream iss(read);

    string line;
    getline(iss, line);
    while (getline(iss, line))
    {
        if (line.compare(0, 26, "ash: Location out of reach") == 0) {
            cerr << "Location out of reach" << endl;
            return -2;
        }
    }

    //Assuming speed of 10, need this delay. If you change your speed, you can change this delay but be very careful
    sleepms(3000);
}

void serialConnectA255::sleepms(int milliseconds)
{
    usleep(milliseconds * 1000);
}

int serialConnectA255::writeString(string str)
{
    //if connected
    if (fd_ != -1)
    {
        int countSent = write(fd_, str.c_str(), str.length());

        //Check if data write was successfull
        if (countSent < 0)
        {
            cerr << "Could not write to the serial port properly." << endl;
            return -1;
        }
    }
    else
    {
        cerr << "Could not send the command over serial port. Check if connected." << endl;
        return -1;
    }

    return 0;
}

int serialConnectA255::readString(string &str)
{
    int countRcv;

    //if connected
    if (fd_ != -1)
    {
        char buf[BUFFER_SIZE + 1] = "";

        str = "";
        int i = 0;
        while ((countRcv = read(fd_, buf, BUFFER_SIZE)) > 0)
        {
            str.append(buf, countRcv);

            //No further data.
            if (countRcv < BUFFER_SIZE)
                break;
        }

        if (countRcv < 0)
        {
            if (errno == EAGAIN) //If there is no data available, the read() returns -1, with errno set to [EAGAIN].
            {
                ROS_WARN("There is no data available to read. Check if connected.");
                return -1;
            }
            else
            {
                cerr << "Could not receive the response over serial port. Check if connected." << endl;
                return -1;
            }
        }
    }
    else
    {
        cerr << "Could not receive the response over serial port. Check if connected." << endl;
        return -1;
    }

    return 0;
}

// Function to go to Ready position which is (0,90,0,0,0) in CRS convention
int serialConnectA255::goReady()
{
    ROS_INFO("Going to 'Ready' position\n");

    int status = writeString("ready\r");
    if (status != 0)
    {
        cerr << "Unable to go to 'ready' position.\n" << endl;
        return -1;
    }
    sleepms(10000);
    return 0;
}

// Function for moving the tool through straight line in the z-direction
int serialConnectA255::goInZ(double desZ)
{
    // Set the joint speed
    ostringstream spConverted;
    spConverted << desZ; // convert from int to str
    string tempZZ = "";
    tempZZ = "wzs";
    tempZZ = tempZZ + " " + spConverted.str() + "\r";
    cout << tempZZ << "\n";

    int status = writeString(tempZZ);
    if (status != 0)
    {
        cerr << "Unable to goInZ" << endl;
        return -1;
    }
    sleepms(3000);
    tempZZ = "";
    return 0;
}

// Functions for opening and closing the gripper
int serialConnectA255::grip_open()
{

    int status = writeString("grip_open\r");
    if (status != 0)
    {
        cerr << "Unable to send 'grip_open'" << endl;
        return -1;
    }
    sleepms(2000);
}

int serialConnectA255::grip_close()
{

    int status = writeString("grip_close\r");
    if (status != 0)
    {
        cerr << "Unable to send 'grip_close'" << endl;
        return -1;
    }
    sleepms(2000);
}
