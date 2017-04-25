#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include "lib2dl.h"
#include "ymbc.h"
using namespace std;

#define D_SHAKO 0.4
#define M_SHAKO 0.15


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

	node_.param("LoopRate", LoopRate, 30);
	node_.param("ScanTopic", ScanTopic, string("/scan"));
	node_.param("ScanTopic", ScanPubTopic, string("/scan_clustered"));

	ros::Rate looprate(LoopRate);
	ros::Subscriber ScanSub = node_.subscribe(ScanTopic, 1, ScanCallback);
	ros::Publisher ScanClusteredPub = node_.advertise<sensor_msgs::LaserScan>(ScanPubTopic, 1);
	ros::Publisher LNVPub = node_.advertise<geometry_msgs::PoseArray>("LNV", 1);

	if (!ymbc.init()) return -1;

	while(ros::ok())
	{
		if(!ScanSubFlag) {
			// Spur_stop();
		}
		else {
			sensor_msgs::LaserScan scan_clustered;
			lib2dl.ScanRegister(scan);
			lib2dl.Clustering(0.015, 0.01, 10);
			vector<Lib2dl::LineData> linelist;
			lib2dl.GetLineList(linelist, 0.003, true);
			for (int i=0; i<linelist.size(); ++i) {
				double distance = linelist[i].b / sqrt(1+linelist[i].a);
			}
		}
		ros::spinOnce();
		looprate.sleep();
	}



			// geometry_msgs::PoseArray posearray;
			// posearray.header = scan.header;
			// lib2dl.LNV();
			// vector<Lib2dl::LNVData> lnvlist;
			// lib2dl.GetLNVList(lnvlist);
			// for (int i=0; i<lnvlist.size(); ++i) {
			// 	geometry_msgs::Pose pose;
			// 	pose.position.x = lnvlist[i].x; 
			// 	pose.position.y = lnvlist[i].y; 
			// 	pose.position.z = 0.0;
			// 	double yaw = atan(linelist[i].a);
			// 	geometry_msgs::Quaternion q;
			// 	quaternionTFToMsg(tf::createQuaternionFromRPY(0.0,0.0,lnvlist[i].theta), q);
			// 	pose.orientation = q;
			// 	posearray.poses.push_back(pose);
			// }
			// LNVPub.publish(posearray);
			// lib2dl.GetClusteredScan(scan_clustered);
			// ScanClusteredPub.publish(scan_clustered);
			// ScanSubFlag = false;

	return 0;
}
