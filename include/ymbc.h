#ifndef YMBC_STRY_
#define YMBC_STRY_

#include <unistd.h>
#include <ypspur.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <string>

void ymbc_ctrlC(int);

class Ymbc
{
	public:
		Ymbc();
		// if init success : return 1   else : return 0
		bool init();
		// control with keyboard (vel, accel, angvel, angaccel);
		void keycon(double, double, double, double);
	private:
		void SetVels();
		double vel;
		double accel;
		double angvel;
		double angaccel;
};

#endif
