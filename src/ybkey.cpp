#include <ros/ros.h>
#include "ymbc.h"
using namespace std;

Ymbc ymbc;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ybkey");
	ros::NodeHandle node_("~");
	
	if (!ymbc.init()) return -1;
	cout<<"ybkey!"<<endl;

	ymbc.keycon(0.3, 1.0, 0.5, 2.0);
	
	return 0;
}
