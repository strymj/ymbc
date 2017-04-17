#include "ymbc.h"
using namespace std;

#define LOOPWAIT_MS 10000
#define BODY_X 0.3
#define BODY_Y 0.2
#define POLE_AX 1.0
#define POLE_AY 0.0
#define POLE_BX 3.0
#define POLE_BY 0.0
#define POLE_MARGIN 0.5
#define GATE_X 2.0
#define GATE_Y 1.5
#define GATE_MARGINE_X 0.4
#define THROUGH_Y 0.5
#define GOAL_MARGINE_Y 0.3


Ymbc ymbc;

int main(int argc, char **argv)
{
	if (!ymbc.init()) return -1;
	cout<<"kadai1 program."<<endl;

	Spur_set_vel(0.3);
	Spur_set_accel(1.0);
	Spur_set_angvel(1.5);
	Spur_set_angaccel(2.0);

	// global flame init
	Spur_set_pos_GL(-BODY_Y/2.0, 0.0, M_PI/2.0);
	
	// poal a
	cout<<"pole a"<<endl;
	Spur_circle_GL(POLE_AX, POLE_AY, -POLE_MARGIN);
	while (!Spur_over_line_GL(POLE_AX, POLE_AY, 0.0)) {
		usleep(LOOPWAIT_MS);
	}
	while (!Spur_over_line_GL(POLE_AX, POLE_AY, M_PI)) {
		usleep(LOOPWAIT_MS);
	}
	while (!Spur_over_line_GL(POLE_AX, POLE_AY, 0.0)) {
		usleep(LOOPWAIT_MS);
	}
	
	// poal b
	cout<<"pole b"<<endl;
	Spur_circle_GL(POLE_BX, POLE_BY, -POLE_MARGIN+BODY_Y/2.0);
	while (!Spur_over_line_GL(POLE_BX, POLE_BY, 0.0)) {
		usleep(LOOPWAIT_MS);
	}
	while (!Spur_over_line_GL(POLE_BX, POLE_BY, M_PI)) {
		usleep(LOOPWAIT_MS);
	}

	// go forward
	cout<<"go forward"<<endl;
	Spur_line_GL(0.0, -POLE_MARGIN, -M_PI);
	while (!Spur_over_line_GL(GATE_X+GATE_MARGINE_X, GATE_Y, M_PI)) {
		usleep(LOOPWAIT_MS);
	}
	

	// gate
	cout<<"gate"<<endl;
	Spur_line_GL(GATE_X, GATE_Y, M_PI/2.0);
	while (!Spur_over_line_GL(GATE_X, GATE_Y+THROUGH_Y, M_PI/2)) {
		usleep(LOOPWAIT_MS);
	}

	// return home
	cout<<"retuen home"<<endl;
	Spur_line_GL(GATE_X, GATE_Y+THROUGH_Y+GOAL_MARGINE_Y, M_PI);
	while (!Spur_over_line_GL(0.0-BODY_X, 0.0, M_PI)) {
		usleep(LOOPWAIT_MS);
	}
	
	Spur_stop();

	return 0;
}
