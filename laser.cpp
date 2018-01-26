#include "laser.h"

#include <iostream>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#include <csignal>

using namespace std;
LaserDistanceMeter Laser = LaserDistanceMeter();

LaserDistanceMeter::LaserDistanceMeter() {
	serial_io = 0;
}

int LaserDistanceMeter::set_laser_interface_attribs (int speed, int parity) {
	memset(&(tty), 0, sizeof(tty));
	if (tcgetattr(fd, &(tty)) != 0) {
		//error_message ("error %d from tcgetattr", errno);
		perror("tcgetattr error :");
		return -1;
	}

	cfsetospeed(&(tty), speed);
	cfsetispeed(&(tty), speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;		// 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;			// disable break processing
	tty.c_lflag = 0;				// no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;				// no remapping, no delays
	tty.c_cc[VMIN]  = 0;			// read doesn't block
	tty.c_cc[VTIME] = 5;			// 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);		// ignore modem controls,
											// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);		// shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &(tty)) != 0) {
		//error_message ("error %d from tcsetattr", errno);
		perror("tcsetattr error :");
		return -1;
	}
	return 0;
}

void LaserDistanceMeter::laser_error(int err) {
	switch (err) {
		case 1: cout << "Er01:VBAT too low, power voltage should >=2.0V" << endl; break;
		case 2: cout << "Er02:Internal error, don't care" << endl; break;
		case 3: cout << "Er03:Module temperature is too low(<-20^C)" << endl; break;
		case 4: cout << "Er04:Module temperature is too high(>+40^C)" << endl; break;
		case 5: cout << "Er05:Target out of measure range" << endl; break;
		case 6: cout << "Er06:Invalid measure result" << endl; break;
		case 7: cout << "Er07:Background light is too strong" << endl; break;
		case 8: cout << "Er08:Laser signal is too weak" << endl; break;
		case 9: cout << "Er09:Laser signal is too strong" << endl; break;
		case 10: cout << "Er10:Hardware fault 1" << endl; break;
		case 11: cout << "Er11:Hardware fault 2" << endl; break;
		case 12: cout << "Er12:Hardware fault 3" << endl; break;
		case 13: cout << "Er13:Hardware fault 4" << endl; break;
		case 14: cout << "Er14:Hardware fault 5" << endl; break;
		case 15: cout << "Er15:Laser signal is not stable" << endl; break;
		case 16: cout << "Er16:Hardware fault 6" << endl; break;
		case 17: cout << "Er17:Hardware fault 7" << endl; break;
	}
}

void LaserDistanceMeter::set_laser_blocking (int should_block) {
	memset (&(tty), 0, sizeof(tty));
	if (tcgetattr(fd, &(tty)) != 0) {
		//error_message ("error %d from tggetattr", errno);
		perror("tcgetattr error :");
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr(fd, TCSANOW, &(tty)) != 0) {
	// error_message ("error %d setting term attributes", errno);
	perror("tcsetattr error :");
	}
}

void LaserDistanceMeter::set_laser_signal() {
	signal(SIGIO, laser_signal_handler);
	fcntl(fd, F_SETOWN, getpid());
	fcntl(fd, F_SETFL, FASYNC);
}

int LaserDistanceMeter::request_serial(char order) {
	memset(buf, '\0', sizeof(buf));
	if (write(fd, &order, 1) == -1) {
		perror("write error:");
		return -1;
	}
	serial_io = 0;
	while (serial_io == 0);
	if (read(fd, buf, sizeof(buf) - sizeof(char)) == -1) {
		perror("read error:");
		return -1;
	}
	//cout << buf;
	serial_io = 0;
	while (serial_io == 0);
	return 0;
}

int LaserDistanceMeter::init(const char port[], int speed, int parity) {
	strcpy(portname, port);
	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	
	if (fd < 0) {
		perror("Open Error: ");
		return -1;
	}

	set_laser_interface_attribs(speed, parity);
	set_laser_blocking(0);
	set_laser_signal();
	return 0;
}

void LaserDistanceMeter::control_laser(char order) {
	if (request_serial(order) == -1) {
		cout << "serial error!" << endl;
		return;
	}
	char *p = strchr(buf, ',') + sizeof(char);
	if (NULL == p) {
		cout << "parsing error" << endl;
		return;
	}
	cout << "Open laser:" << p << endl;
}

void LaserDistanceMeter::get_laser_value(char order) {
	if (request_serial('D') == -1) {
		cout << "serial error!" << endl;
		return;
	}		
	char *p = strchr(buf, ':') + sizeof(char);
	if (NULL == p) {
		cout << "parsing error!" << endl;
		return;
	}
	char *end = strchr(buf, 'm');
	if (NULL == end) {
		p = p + sizeof(char) * 2;
		if ((end = strchr(buf, '!')) != NULL) {
			*end = '\0';
			laser_error(atoi(p));
		}	
		return;
	}
	value = atof(p);
}

int LaserDistanceMeter::get_laser_state() {

}

void laser_signal_handler(int signum) {
	Laser.serial_io = 1;
}



