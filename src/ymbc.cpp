#include "ymbc.h"
#include "kbhit.h"
using namespace std;

void ctrlC(int aStatus)
{
	Spur_stop();
	signal(SIGINT, NULL);
	exit(aStatus);
}

int Ymbc::init()
{
	signal(SIGINT, ctrlC);
	if (!Spur_init()) {
		fprintf(stderr, "ERROR : cannot open spur\n");
		cout<<"cannot open spur."<<endl;
		return 0;
	}
	return 1;
}

void Ymbc::keycon(double vel, double accel, double angvel, double angaccel)
{
	int LOOPWAIT_US = 10000;

	cout<<"control yamabiko with keyboard!!!"<<endl;
	cout<<"##### usage #####"<<endl;
	cout<<"o       : stop"<<endl; 
	cout<<"<Up>    : straight"<<endl; 
	cout<<"<Down>  : back"<<endl; 
	cout<<"<PgUp>  : go left"<<endl; 
	cout<<"<PgDn>  : go right"<<endl; 
	cout<<"<Left>  : turn left"<<endl; 
	cout<<"<Right> : turn right"<<endl; 
	enum {STRAIGHT=0x41, BACK=0x42, GO_LEFT=0x35, GO_RIGHT=0x36, TURN_REFT=0x44, TURN_RIGHT=0x43, STOP='o'};
	while (1) {
		if (kbhit()) {
			short int key;
			key = getchar();
			printf("%X\n",key);
			switch(key){
				case STRAIGHT:
					cout<<"   straight"<<endl;
					Spur_vel(vel, 0.0);
					break;
				case BACK:
					cout<<"   back"<<endl;
					Spur_vel(-vel, 0.0);
					break;
				case GO_LEFT:
					cout<<"   go_left"<<endl;
					Spur_vel(vel, angvel);
					break;
				case GO_RIGHT:
					cout<<"   go_right"<<endl;
					Spur_vel(vel, -angvel);
					break;
				case TURN_REFT:
					cout<<"   turn_left"<<endl;
					Spur_vel(0.0, angvel);
					break;
				case TURN_RIGHT:
					cout<<"   turn_right"<<endl;
					Spur_vel(0.0, -angvel);
					break;
				case STOP:
					cout<<"   stop"<<endl;
					Spur_stop();
					break;
				default:;
			}
		}
		usleep(LOOPWAIT_US);
	}
}

