#include "ymbc.h"

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


