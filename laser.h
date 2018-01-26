#ifndef _LASER_H_
#define _LASER_H_

#include <termios.h>
#include <signal.h>

class LaserDistanceMeter {
private:
	struct termios tty;
	//static char buf[255];
	int fd;
	double value_history[10];
	char buf[128];
	char portname[128];

	int set_laser_interface_attribs(int speed, int parity);  //set tty interface
	void laser_error(int err);
	void set_laser_blocking (int should_block);  //set comunication block
	void set_laser_signal();  //set SIGIO signal
	int request_serial(char ch);  //send serial, receive & write buffer

public:
	double value;
	unsigned char serial_io;  // Used as indicator. **DO NOT MODIFY**

	LaserDistanceMeter();

	///////////////////////////////////
	/********Methods For User*********/
	///////////////////////////////////
	int init(const char port[], int speed, int parity);
	void control_laser(char order);  //if order == 'O': turn the laser on/'C': turn the laser off
	void get_laser_value(char order);  //if order == 'D': slow measure mode/command 'F': fast measure mode
	int get_laser_state();  // get status
};

void laser_signal_handler(int signum);
extern LaserDistanceMeter Laser;

#endif
