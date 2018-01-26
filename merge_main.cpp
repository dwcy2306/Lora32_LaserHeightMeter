#include <iostream>
#include <math.h>
#include <csignal>
#include <pthread.h>
#include "MPU9255.h"
#include "laser.h"

using namespace std;

/* Global Variables */
char run = 1;
char resUpdated = 0;
/* Global Variables end */
/* Function Prototypes */
double calcDistance(double rawDist);

void sigHandler(int signum);

void* updateLaser(void*);
void* updateMPU(void*);
/* Function Prototypes end */
/////////////////////////////
/* Main start */
int main(void) {
	int tmp;
	pthread_t p_laser;
	pthread_t p_mpu;
	signal(SIGINT, sigHandler);

	tmp = Laser.init("/dev/ttyUSB0", B19200, 0);
	cout << "Laser init complete: " << tmp << endl;

	Mpu.initialize();
	cout << "MPU init complete" << endl;

	Mpu.setAlpha(0.98);  // Default is 0.96

	pthread_create(&p_laser, NULL, updateLaser, NULL);
	pthread_create(&p_mpu, NULL, updateMPU, NULL);

	while (run) {
		if (resUpdated) {
			cout << fixed << Mpu.Angles[0] << "\t" << fixed << Mpu.Angles[1] << "\t" << fixed << Laser.value << "\t" << fixed << calcDistance(Laser.value) << endl;
			resUpdated = 0;
		}
	}

	pthread_join(p_laser, NULL);
	pthread_join(p_mpu, NULL);
	cout << "Program teminated" << endl;
	return 0;
}
/* Main end */
///////////////////////////////

double calcDistance(double rawDist) {
	double result;

	result = sqrt((1 - pow(sin(Mpu.angles[0] * M_PI / 180), 2)) * pow(rawDist, 2) * pow(cos(Mpu.angles[1] * M_PI / 180), 2));
	return result;
}

void sigHandler(int signum) {
	cout << "Signal Received" << endl;
	run = 0;
}

void* updateLaser(void*) {
	while (run) {
		Laser.get_laser_value('F');
		resUpdated = 1;
	}
}

void* updateMPU(void*) {
	Mpu.initDT();
	while (run) {
		Mpu.getResult();
		Mpu.calcDT();  // MUST place this at the end of loop
	}
}



