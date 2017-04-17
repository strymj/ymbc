#include "ymbc.h"
using namespace std;

Ymbc ymbc;

int main(int argc, char **argv)
{
	if (!ymbc.init()) return -1;
	cout<<"ybkey!"<<endl;

	ymbc.keycon(0.3, 1.0, 1.0, 2.0);
	
	return 0;
}
