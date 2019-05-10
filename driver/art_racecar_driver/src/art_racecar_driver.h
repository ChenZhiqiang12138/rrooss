
//
// Created by Steven Zhang on 18-12-14.
//

#ifndef ART_RACECAR_DRIVER
#define ART_RACECAR_DRIVER
#include <stdint.h>
#include <unistd.h>

#if defined(__cplusplus)
extern "C" {
#endif

/****************协议****************/

#define HEAD     0xAA
#define TAIL     0x55



int Open_Serial_Dev(char *dev);
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
int art_racecar_init(int speed,char *dev);
unsigned char check_uint(uint16_t data); //计算uint16的和校验

unsigned char send_cmd(uint16_t motor_pwm,uint16_t servo_pwm);
void print_send_buff(struct cmd);
static int read_port (char *data, int datalength);




struct cmd {
    unsigned char null;
    unsigned char H;
    uint16_t Motor_PWM;
    uint16_t Servo_PWM;
    unsigned char CS;
    unsigned char T;
};

union num_trans_uint16{
    unsigned char num_char[2];
    uint16_t num_int16;
};

union send_cmd{
    struct cmd cmd0;
    unsigned char data[8];
};

#if defined(__cplusplus)
}
#endif
#endif