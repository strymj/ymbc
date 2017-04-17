#ifndef YMBC_STRY_
#define YMBC_STRY_

#include <unistd.h>
#include <ypspur.h>
#include <stdlib.h>
#include <signal.h>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <string>

void ctrlC(int);

class Ymbc
{
	public:
		// if init success : return 1   else : return 0
		bool init();
		// control with keyboard (vel, accel, angvel, angaccel);
		void keycon(double, double, double, double);
	private:
};

#endif
