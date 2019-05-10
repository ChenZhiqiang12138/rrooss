//
// Created by Steven Zhang on 18-12-14.
// art racecar
//

#include "../include/art_racecar_driver.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>

int i;
int vel,angle;

void send_msg(int signum)
{
    printf("num...%d\n",i++);
	send_cmd(uint16_t(vel++),uint16_t(angle));
}

void setTimer(int seconds, int useconds)
{
	struct sigaction sa;
    struct itimerval timer;
	/* Install timer_handler as the signal handler for SIGVTALRM. */
    memset(&sa, 0, sizeof (sa));
    sa.sa_handler = &send_msg;
	sigaction (SIGVTALRM, &sa, NULL);
	/* Configure the timer to expire after 100 msec... */
    timer.it_value.tv_sec = seconds;
    timer.it_value.tv_usec = useconds;
	/* ... and every 100 msec after that. */
    timer.it_interval.tv_sec = seconds;
    timer.it_interval.tv_usec = useconds;
	/* Start a virtual timer. It counts down whenever this process is executing. */
    setitimer (ITIMER_VIRTUAL, &timer, NULL);

}

int main(int argc, char** argv)
{
	char data[] = "/dev/ttyUSB0";
	if(art_racecar_init(38400,data) < 0)
	{
		printf("can't find %s\n",data);
		return 0;
    
	}
			
	setTimer(0,50000);//20Hz
	while(1)
    {

    }
 	
	return 0;
}
