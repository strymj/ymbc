#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "lib2dl.h"
#include "ymbc.h"
using namespace std;

static double LIDAR_OFFSET_X = 0.2;
static double LIDAR_OFFSET_Y = 0.0;
static double BODY_TILT = M_PI/6;
static double LINE1_DIST = 0.4;
static double LINE1_MARGIN = 0.1;
static int LINE1_THRE = 30;
static double SEARCH_ANG = -M_PI/3;

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

int LineDetection(Lib2dl::Line& line, double distm, double distM, double thetam, double thetaM, int threshold)
{
	bool ans = false;
	lib2dl.ScanRegister(scan);
	lib2dl.ImgProt();
	lib2dl.ImgShow("scan");
	vector<Lib2dl::Line> lines;
	lib2dl.GetHoughLinesP(lines, 2.0, M_PI/360, threshold, 0.15, 0.1);
	lib2dl.ImgShow("line");
	// cout<<"line num = "<<lines.size()<<endl;

	for (int i=0; i<lines.size(); ++i) {
		cout<<"line dist  = "<<lines[i].distance<<endl; 
		double theta;
		if (lines[i].theta>0) theta = lines[i].theta - M_PI/2;
		else theta = lines[i].theta + M_PI/2;
		cout<<"line theta = "<<theta<<endl; 
		if(distm<lines[i].distance && lines[i].distance<distM
				&& thetam < theta && theta<thetaM) {
			lines[i].theta = theta;
			line = lines[i];
			ans = true;
			break;
		}
	}
	return ans;
}

// cv::Mat image;
// lib2dl.GetImage(image);
// sensor_msgs::ImagePtr image_msg;
// // image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
// image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
// ImagePub.publish(image_msg);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "kadai2");
	ros::NodeHandle node_("~");

	signal(SIGINT, ctrlC);

	int LoopRate;
	string ScanTopic, ScanCTopic, ScanImgTopic;

	node_.param("LoopRate", LoopRate, 30);
	node_.param("ScanTopic", ScanTopic, string("/scan"));
	node_.param("ScanTopic", ScanCTopic, string("/scan_clustered"));
	node_.param("ScanImgTopic", ScanImgTopic, string("/scan_image"));

	ros::Rate looprate(LoopRate);
	ros::Subscriber ScanSub = node_.subscribe(ScanTopic, 1, ScanCallback);
	ros::Publisher ScanClusteredPub = node_.advertise<sensor_msgs::LaserScan>(ScanCTopic, 1);
	image_transport::ImageTransport it(node_);
	image_transport::Publisher ImagePub = it.advertise(ScanImgTopic, 1);

	if (!ymbc.init()) {
		ros::shutdown();
		return -1;
	}
	
	lib2dl.SetLidarFrame(0.15, 0.0);
	lib2dl.ImgProtConfig(240, 0.8);


	// while(ros::ok())
	// {
	// 	if (ScanSubFlag) {
	// 		Lib2dl::Line line0;
	// 		cout<<LineDetection(line0, LINE1_DIST-LINE1_MARGIN, LINE1_DIST+LINE1_MARGIN,
	// 				-BODY_TILT, BODY_TILT, LINE1_THRE)<<endl;
	// 	}
	// 	ScanSubFlag = false;
	// 	ros::spinOnce();
	// 	looprate.sleep();
	// }

	// // rotate -90 [deg]
	// cout<<"[spur]  Rotate -90[deg]."<<endl;
	// Spur_set_angvel(0.8);
	// Spur_spin_FS(-M_PI/2);
	// sleep(3);

	// carbox detection
	Lib2dl::Line line2;
	bool line2_exist = false;
	lib2dl.ImgProtConfig(240, 1.5);
	while(ros::ok())
	{
		if (ScanSubFlag) {
			line2_exist = LineDetection(line2, LINE1_DIST-LINE1_MARGIN, LINE1_DIST+LINE1_MARGIN,
					-BODY_TILT, BODY_TILT, 20);
			//if (line1_exist) break;
		}
		ScanSubFlag = false;
		ros::spinOnce();
		looprate.sleep();
	}

	// line1 detection
	Lib2dl::Line line1;
	bool line1_exist = false;
	lib2dl.ImgProtConfig(240, 1.5);
	while(ros::ok())
	{
		if (ScanSubFlag) {
			line1_exist = LineDetection(line1, LINE1_DIST-LINE1_MARGIN, LINE1_DIST+LINE1_MARGIN,
					-BODY_TILT, BODY_TILT, LINE1_THRE);
			if (line1_exist) break;
		}
		ScanSubFlag = false;
		ros::spinOnce();
		looprate.sleep();
	}

	// confronting
	cout<<"[spur]  Rotate."<<endl;
	Spur_set_angvel(0.4);
	Spur_spin_FS(M_PI/2 + line1.theta);
	sleep(6);

	// go forward
	Spur_line_FS(0.0, 0.0, 0.0);
	while(ros::ok())
	{
		if (ScanSubFlag) {
			line1_exist = LineDetection(line1, LINE1_DIST-LINE1_MARGIN, LINE1_DIST+LINE1_MARGIN,
					-BODY_TILT, BODY_TILT, LINE1_THRE);
			if (line1_exist) break;
		}
		ScanSubFlag = false;
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
