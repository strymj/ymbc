#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "lib2dl.h"
#include "ymbc.h"
using namespace std;

#define POLE_D 0.115
#define POLW_M 0.03

Ymbc ymbc;
Lib2dl lib2dl;
sensor_msgs::LaserScan scan;
bool ScanSubFlag = false;

void ctrlC(int aStatus)
{
	Spur_stop();
	signal(SIGINT, NULL);
	ros::shutdown();
}

void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	scan = *msg;
	ScanSubFlag = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kadai1");
	ros::NodeHandle node_("~");
	
	signal(SIGINT, ctrlC);

	int LoopRate;
	string ScanTopic, ScanPubTopic;

	node_.param("RoopRate", LoopRate, 30);
	node_.param("ScanTopic", ScanTopic, string("/scan"));
	node_.param("ScanTopic", ScanPubTopic, string("/scan_clustered"));

	ros::Rate looprate(LoopRate);
	ros::Subscriber ScanSub = node_.subscribe(ScanTopic, 1, ScanCallback);
	ros::Publisher ScanClusteredPub = node_.advertise<sensor_msgs::LaserScan>(ScanPubTopic, 1);

	if (!ymbc.init()) return -1;

	while(ros::ok())
	{
		if(!ScanSubFlag) {
			Spur_stop();
		}
		else {
			sensor_msgs::LaserScan scan_clustered;
			lib2dl.ScanRegister(scan);
			lib2dl.Clustering(0.03, 30);
			lib2dl.GetClusteredScan(scan_clustered);
			vector<Lib2dl::CircleData> polelist;
			lib2dl.GetCircleList(polelist, 0.0, 1.0, 3.0);
			for(int i=0; i<polelist.size(); ++i) {
				cout<<"x = "<<polelist[i].x<<endl;
				cout<<"y = "<<polelist[i].y<<endl;
				cout<<"d = "<<polelist[i].r*2<<endl;
				cout<<"e = "<<polelist[i].ErrorBar<<endl;
				cout<<endl;
			}
			ScanClusteredPub.publish(scan_clustered);
			ScanSubFlag = false;
		}
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
