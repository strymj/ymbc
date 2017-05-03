#include "ymbc.h"
#include "kbhit.h"
using namespace std;

bool INTERRUPT = false;

void ymbc_ctrlC(int aStatus)
{
	Spur_stop();
	signal(SIGINT, NULL);
	INTERRUPT = true;
	//exit(aStatus);
}

Ymbc::Ymbc():
	vel(0.3),
	accel(1.5),
	angvel(0.5),
	angaccel(2.5)
{

}

bool Ymbc::init()
{
	signal(SIGINT, ymbc_ctrlC);
	if (!Spur_init()) {
		fprintf(stderr, "ERROR : cannot open spur\n");
		cout<<"cannot open spur."<<endl;
		return false;
	}

	Spur_set_vel(vel);
	Spur_set_accel(accel);
	Spur_set_angvel(angvel);
	Spur_set_angaccel(angaccel);

	return true;
}

void Ymbc::SetVels() 
{
	Spur_set_vel(vel);
	Spur_set_accel(accel);
	Spur_set_angvel(angvel);
	Spur_set_angaccel(angaccel);
}

void Ymbc::keycon(double i_vel, double i_accel, double i_angvel, double i_angaccel)
{
	int LOOPWAIT_US = 10000;

	vel = i_vel;
	accel = i_accel;
	angvel = i_angvel;
	accel = i_angaccel;
	SetVels();

	cout<<"control yamabiko with keyboard!!!"<<endl;
	cout<<"##### usage #####"<<endl;
	cout<<"\\       : stop"<<endl; 
	cout<<"<Up>    : straight"<<endl; 
	cout<<"<Down>  : back"<<endl; 
	cout<<"<PgUp>  : go left"<<endl; 
	cout<<"<PgDn>  : go right"<<endl; 
	cout<<"<Left>  : turn left"<<endl; 
	cout<<"<Right> : turn right"<<endl; 
	enum {STRAIGHT=0x41, BACK=0x42, GO_LEFT=0x35, GO_RIGHT=0x36, TURN_REFT=0x44, TURN_RIGHT=0x43, STOP='\\', by_0='z', by_3='a', by_6='q'};
	while (!INTERRUPT) {
		if (kbhit()) {
			short int key;
			key = getchar();
			// check key char
			//printf("%X\n",key);
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
				case by_0:
					cout<<"   default velocity"<<endl;
					vel = i_vel;
					angvel = i_angvel;
					SetVels();
					break;
				case by_3:
					cout<<"   velocity *= 1.3"<<endl;
					vel = i_vel * 1.3;
					angvel = i_angvel * 1.3;
					SetVels();
					break;
				case by_6:
					cout<<"   velocity *= 1.6"<<endl;
					vel = i_vel * 1.6;
					angvel = i_angvel * 1.6;
					SetVels();
					break;
				default:;
			}
		}
		usleep(LOOPWAIT_US);
	}
}

