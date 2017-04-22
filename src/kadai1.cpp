#include <ros/ros.h>
#include "ymbc.h"
using namespace std;

#define LOOPWAIT_MS 10000
#define BODY_X 0.3
#define BODY_Y 0.25
#define POLE_AX 1.0
#define POLE_AY 0.0
#define POLE_BX 3.0
#define POLE_BY 0.0
#define POLE_MARGIN 0.6
#define GATE_MARGINE_X 0.6
#define GATE_X 2.0
#define GATE_Y 1.5
#define GATE_WIDTH 0.45
#define GOAL_MARGINE_Y 0.4


Ymbc ymbc;

void ctrlC(int aStatus)
{
	Spur_stop();
	signal(SIGINT, NULL);
	exit(aStatus);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kadai1");
	ros::NodeHandle node_("~");
	
	if (!ymbc.init()) return -1;
	signal(SIGINT, ctrlC);
	cout<<"kadai1 program."<<endl;
	
	Spur_set_vel(0.5);
	Spur_set_angvel(2.5);
	
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
	while (!Spur_over_line_GL(GATE_X, GATE_Y+GATE_WIDTH, M_PI/2)) {
		usleep(LOOPWAIT_MS);
	}

	// return home
	cout<<"retuen home"<<endl;
	Spur_line_GL(GATE_X, GATE_Y+GATE_WIDTH+GOAL_MARGINE_Y, M_PI);
	while (!Spur_over_line_GL(0.0, 0.0, M_PI)) {
		usleep(LOOPWAIT_MS);
	}
	Spur_set_vel(0.2);
	while (!Spur_over_line_GL(0.0-BODY_X, 0.0, M_PI)) {
		usleep(LOOPWAIT_MS);
	}
	
	Spur_stop();

	return 0;
}
