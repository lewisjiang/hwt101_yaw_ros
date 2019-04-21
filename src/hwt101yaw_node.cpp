#include<stdio.h>
#include<stdlib.h>
#include<fcntl.h>
#include<unistd.h>
#include<assert.h>
#include<termios.h>
#include<string.h>
#include<sys/time.h>
#include<time.h>
#include<sys/types.h>
#include<errno.h>

#include <string>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/Imu.h"
#include "std_msgs/Time.h"
#include "ros/ros.h"


// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <Eigen/Dense>

class jy901b_imu
{
private:
    ros::NodeHandle n;
    // ros::Subscriber sub;
    ros::Publisher pub1;
    int loop_rate;
    int ret;
    int fd;

    bool data_read_from_imu;

    float Angle[3], angularV[3];
    int seq;
    int BAUD;

public:
    jy901b_imu(int argc, char** argv) {
        loop_rate = 250;
        seq = 0;
        // BAUD = 9600;
        BAUD = 115200;

        pub1 = n.advertise<sensor_msgs::Imu>("yaw_and_z_angle", 1);
        data_read_from_imu = false;

    }
    int uart_open(int fd, const char *pathname)
    {
        fd = open(pathname, O_RDWR | O_NOCTTY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return (-1);
        }
        else
            printf("open %s success!\n", pathname);
        if (isatty(STDIN_FILENO) == 0)
            printf("standard input is not a terminal device\n");
        else
            printf("isatty success!\n");
        return fd;
    }

    int uart_set(int fd, int nSpeed, int nBits, char nEvent, int nStop)
    {
        struct termios newtio, oldtio;
        if  ( tcgetattr( fd, &oldtio)  !=  0) {
            perror("SetupSerial 1");
            printf("tcgetattr( fd,&oldtio) -> %d\n", tcgetattr( fd, &oldtio));
            return -1;
        }
        bzero( &newtio, sizeof( newtio ) );
        newtio.c_cflag  |=  CLOCAL | CREAD;
        newtio.c_cflag &= ~CSIZE;
        switch ( nBits )
        {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
        }
        switch ( nEvent )
        {
        case 'o':
        case 'O':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'e':
        case 'E':
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'n':
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            break;
        }

        /*设置波特率*/

        switch ( nSpeed )
        {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        }
        if ( nStop == 1 )
            newtio.c_cflag &=  ~CSTOPB;
        else if ( nStop == 2 )
            newtio.c_cflag |=  CSTOPB;
        newtio.c_cc[VTIME]  = 0;
        newtio.c_cc[VMIN] = 0;
        tcflush(fd, TCIFLUSH);






        if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
        {
            perror("com set error");
            return -1;
        }
        printf("set done!\n");
        return 0;
    }

    int uart_close(int fd)
    {
        assert(fd);
        close(fd);

        return 0;
    }

    int send_data(int  fd, char *send_buffer, int length)
    {
        length = write(fd, send_buffer, length * sizeof(unsigned char));
        return length;
    }
    int recv_data(int fd, char* recv_buffer, int length)
    {
        length = read(fd, recv_buffer, length);
        return length;
    }

    void ParseData(char chr)
    {
        static char chrBuf[100];
        static unsigned char chrCnt = 0;
        signed short sData[4];
        unsigned char i;

        time_t now;
        chrBuf[chrCnt++] = chr;
        if (chrCnt < 11) return;

        if ((chrBuf[0] != 0x55) || ((chrBuf[1] & 0x50) != 0x50)) {printf("Error:%x %x\r\n", chrBuf[0], chrBuf[1]); memcpy(&chrBuf[0], &chrBuf[1], 10); chrCnt--; return;}

        memcpy(&sData[0], &chrBuf[2], 8);
        switch (chrBuf[1])
        {


        case 0x52:
            for (i = 0; i < 3; i++) angularV[i] = (float)sData[i] / 32768.0 * 2000.0 / 180 * M_PI;
            printf("\nw:%7.3f %7.3f %7.3f ", angularV[0], angularV[1], angularV[2]);
            break;
        case 0x53:
            for (i = 0; i < 3; i++) Angle[i] = (float)sData[i] / 32768.0 * M_PI;
            printf("\nA:%7.3f %7.3f %7.3f ", Angle[0], Angle[1], Angle[2]);
            break;

        }
        chrCnt = 0;
    }

    void run_node() {
        char r_buf[1024];
        bzero(r_buf, 1024);
        ros::Rate loop_rate_(loop_rate);

        fd = uart_open(fd, "/dev/ttyUSB0"); /*串口号/dev/ttySn,USB口号/dev/ttyUSBn */
        if (fd == -1)
        {
            fprintf(stderr, "uart_open error\n");
            exit(EXIT_FAILURE);
        }

        if (uart_set(fd, BAUD, 8, 'N', 1) == -1)
        {
            fprintf(stderr, "uart set failed!\n");
            exit(EXIT_FAILURE);
        }
        // set z-axis to zero
        unsigned char cmd[] = {0xFF, 0xAA, 0x76, 0x00, 0x00, '\r'};
        int n_written = 0;
        n_written = write( fd, cmd, sizeof(cmd) - 1);

        while (ros::ok()) {
            ret = recv_data(fd, r_buf, 11 * 2);
            if (ret == -1)
            {
                fprintf(stderr, "uart read failed!\n");
                exit(EXIT_FAILURE);
            }
            for (int i = 0; i < ret; i++) {
                ParseData(r_buf[i]);
                if (i == ret - 1)
                    data_read_from_imu = true;
            }
            // usleep(1000);

            if (data_read_from_imu) {
                timespec now;
                clock_gettime(CLOCK_REALTIME, &now);

                sensor_msgs::Imu pimu;
                pimu.header.seq = seq++;
                pimu.header.stamp.sec = now.tv_sec;
                pimu.header.stamp.nsec = now.tv_nsec;
                pimu.header.frame_id = "yaw_z_data";

                tf2::Quaternion quatern;
                quatern.setRPY(0,0,Angle[2]);

                pimu.orientation.x = quatern.x();
                pimu.orientation.y = quatern.y();
                pimu.orientation.z = quatern.z();
                pimu.orientation.w = quatern.w();

                pimu.angular_velocity.x = angularV[0];
                pimu.angular_velocity.y = angularV[1];
                pimu.angular_velocity.z = angularV[2];

                pimu.linear_acceleration.x = 0;
                pimu.linear_acceleration.y = 0;
                pimu.linear_acceleration.z = 0;

                for (int i = 0; i < 9; i++) {
                    pimu.orientation_covariance[i] = -1.0;
                    pimu.angular_velocity_covariance[i] = -1.0;
                    pimu.linear_acceleration_covariance[i] = -1.0;
                }


                pub1.publish(pimu);
                loop_rate_.sleep();
                // ros::spinOnce();
                data_read_from_imu = false;
            }
        }

        ret = uart_close(fd);
        if (ret == -1)
        {
            fprintf(stderr, "uart_close error\n");
            exit(EXIT_FAILURE);
        }

        exit(EXIT_SUCCESS);
    }

// ROS_INFO("LOGGING <%10f>", msg->header.stamp.toSec());



};


int main(int argc, char** argv) {
    ros::init(argc, argv, "jy901b_imu");
    jy901b_imu c(argc, argv);
    c.run_node();
    return 0;
}