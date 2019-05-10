
//
// Created by Steven Zhang on 18-12-14.
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include "art_racecar_driver.h"

#define BigLittleSwap16(A)        ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))
#define  Cheack_Uint(Data)         ((((uint16_t)(Data) & 0xff00) >> 8) + ((uint16_t)(Data) & 0x00ff))
unsigned char *ptr = NULL;
volatile struct cmd_long (*pl) = NULL;
volatile struct cmd_short_0 (*ps0) = NULL;
volatile struct cmd_short_1 (*ps1) = NULL;
int fd;
static fd_set rd;

int Open_Serial_Dev(char *dev)
{
    fd = open(dev,O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1)
    {
        printf("can't not open the Serial port! \n");
        return -1;
    }
    else
    {
        printf("Open Serial port success! \n");
        return 1;
    }

}

int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
/*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/
    if  ( tcgetattr( fd,&oldtio)  !=  0) {
        perror("SetupSerial 1");
        printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio));
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
/*步骤一，设置字符大小*/
    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
/*设置停止位*/
    switch( nBits )
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }
/*设置奇偶校验位*/
    switch( nEvent )
    {
        case 'o':
        case 'O': //奇数
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'e':
        case 'E': //偶数
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'n':
        case 'N':  //无奇偶校验位
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            break;
    }
    /*设置波特率*/
    switch( nSpeed )
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
        case 38400:
            cfsetispeed(&newtio, B38400);
            cfsetospeed(&newtio, B38400);
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
/*设置停止位*/
    if( nStop == 1 )
        newtio.c_cflag &=  ~CSTOPB;
    else if ( nStop == 2 )
        newtio.c_cflag |=  CSTOPB;
/*设置等待时间和最小接收字符*/
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
/*处理未接收字符*/
    tcflush(fd,TCIFLUSH);
/*激活新配置*/
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}

int art_racecar_init(int speed,char *dev)
{
    //char *dev = "/dev/ttyUSB0";
    if(Open_Serial_Dev(dev) == -1){
        return -1;
    }
    else{
        if(fcntl(fd,F_SETFL,FNDELAY) < 0)//非阻塞，覆盖前面open的属性
        {
            printf("fcntl failed\n");
        }
        else{
            printf("fcntl=%d\n",fcntl(fd,F_SETFL,FNDELAY));
        }
        set_opt(fd,speed,8,'N',1);
    }
}


unsigned char check_uint(uint16_t data)
{
    union num_trans_uint16 num;
    num.num_int16 = data;
    return num.num_char[0] + num.num_char[1];
}




unsigned char send_cmd(uint16_t motor_pwm,uint16_t servo_pwm)
{
    struct cmd buff;
    buff.null       =   NULL;
    buff.H          =   HEAD;
    buff.Motor_PWM  =   motor_pwm;
    buff.Servo_PWM  =   servo_pwm;
    buff.CS         =   check_uint(motor_pwm) + check_uint(servo_pwm);
    buff.T          =   TAIL;
    write(fd,&buff,8);
    //printf("%02x ",buff.CS);
    print_send_buff(buff);
    //printf("%02x ",buff);
    return buff.CS;
}

void print_send_buff(struct cmd cmd)
{
    union send_cmd data;
    data.cmd0 = cmd;
    printf("the send buff is:\n");

    for (int i = 0; i < 8; i++)
    {
        printf("%02x ",data.data[i]);
    }
    printf("\n");
    printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
}



static int read_port (char *data, int datalength)
{
    int retval = 0;
    int read_len;
    FD_ZERO (&rd);
    FD_SET (fd, &rd);
    retval = select (fd+1, &rd, NULL, NULL, NULL);        //Linux下用select查询串口数据
    if (retval>0) {
        if(FD_ISSET(fd,&rd))
        {
            read_len = read(fd,data,datalength);
            //tcflush(fd,TCIOFLUSH);
            return read_len;
            //return(read(fd,data,datalength));
            //printf("%d\n",read(fd,data,datalength));
            //printf("%s\n",data);

            tcflush(fd,TCIOFLUSH);
        }
    }
    else
        return (-1);
}


