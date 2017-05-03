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
static double ITA_MIN = 0.35;
static double ITA_MAX = 0.55;

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

void LineDetection(vector<Lib2dl::Line>& linelist,
		double distm, double distM,
		double thetam, double thetaM,
		double lengthm, double lengthM,
		int threshold, bool rotate90 = false)
{
	linelist.clear();
	lib2dl.ScanRegister(scan);
	lib2dl.ImgPlot();
	lib2dl.ImgShow("scan");
	vector<Lib2dl::Line> lines;
	lib2dl.GetHoughLinesP(lines, 2.0, M_PI/720, threshold, 0.15, 0.1);
	lib2dl.ImgShow("line");
	// cout<<"line num = "<<lines.size()<<endl;

	for (int i=0; i<lines.size(); ++i) {
		double theta;
		if (rotate90) {
			if (lines[i].theta>0) theta = lines[i].theta - M_PI/2;
			else theta = lines[i].theta + M_PI/2;
			lines[i].theta = theta;
		}
		// cout<<"line dist  = "<<lines[i].distance<<endl; 
		// cout<<"line length  = "<<lines[i].length<<endl; 
		// cout<<"line theta = "<<lines[i].theta<<endl; 
		if(distm<lines[i].distance && lines[i].distance<distM
				&& thetam < lines[i].theta && lines[i].theta<thetaM
				&& lengthm < lines[i].length && lines[i].length < lengthM) {
			linelist.push_back(lines[i]);
		}
	}
}

// cv::Mat image;
// lib2dl.GetImage(image);
// sensor_msgs::ImagePtr image_msg;
// // image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
// image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
// ImagePub.publish(image_msg);

// front calc dist
double front_dist()
{
	int cn = scan.ranges.size() / 2;
	int cn_wid = 3;
	double dist = 0;
	for (int i=cn-cn_wid; i<=cn+cn_wid; ++i) {
		if(!isnan(scan.ranges[i]) && !isinf(scan.ranges[i])) {
			dist += scan.ranges[i];
		}
	}
	dist /= 2*cn_wid + 1;

	return dist;
}


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

	// carbox detection
	vector<Lib2dl::Line> carbox_endline;
	Lib2dl::Point plotcenter = {0.7, 0.0};
	lib2dl.ImgPlotConfig(120, 0.8, plotcenter);
	Spur_set_vel(0.1);
	bool sleeped = false;
	while(ros::ok())
	{
		if (ScanSubFlag) {
			// LineDetection(carbox_endline, 0.2, 1.5, -M_PI/6, M_PI/6, ITA_MIN, ITA_MAX, 20, false);
			// if (carbox_endline.size() == 2) {
			// 	double cbx_x = (carbox_endline[0].bgn.x + carbox_endline[0].end.x
			// 			+ carbox_endline[1].bgn.x + carbox_endline[1].end.x) / 4;
			// 	double cbx_y = (carbox_endline[0].bgn.y + carbox_endline[0].end.y
			// 			+ carbox_endline[1].bgn.y + carbox_endline[1].end.y) / 4;
			// 	double cbx_theta = (carbox_endline[0].theta + carbox_endline[1].theta) / 2;
			// 	cout<<"cbx_x = "<<cbx_x<<endl;
			// 	cout<<"cbx_y = "<<cbx_y<<endl;
			// 	cout<<"cbx_t = "<<cbx_theta<<endl;
			// 	Spur_line_FS(cbx_x, cbx_y, cbx_theta);
			// }
			LineDetection(carbox_endline, 0.2, 1.5, -M_PI/6, M_PI/6, ITA_MIN, ITA_MAX, 15, true);
			cout<<"line_num = "<<carbox_endline.size()<<endl;
			for (int i=0; i<carbox_endline.size(); ++i) {
				cout<<"line dist  = "<<carbox_endline[i].distance<<endl; 
				cout<<"line length  = "<<carbox_endline[i].length<<endl; 
				cout<<"line theta = "<<carbox_endline[i].theta<<endl; 
			}
			cout<<endl<<endl;
			if (carbox_endline.size() == 1) {
				double cbx_x = (carbox_endline[0].bgn.x + carbox_endline[0].end.x) / 2;
				double cbx_y = (carbox_endline[0].bgn.y + carbox_endline[0].end.y) / 2;
				double cbx_theta = carbox_endline[0].theta;
				Spur_line_FS(cbx_x, cbx_y, cbx_theta);
				if (!sleeped) {
					sleep(4);
					sleeped = true;
				}
			}

			if (front_dist()<0.14) {
				Spur_stop();
				cout<<"finish!"<<endl;
				break;
			}
		}
		ScanSubFlag = false;
		ros::spinOnce();
		looprate.sleep();
	}

	cout<<"sleep 20[sec]"<<endl;
	sleep(20);

	cout<<"[kadai2 program]  start."<<endl;

	// line1 detection
	cout<<"[step1]  Line_detection."<<endl;
	vector<Lib2dl::Line> wall;
	lib2dl.ImgPlotConfig(240, 1.5);
	while (ros::ok()) {
		if (ScanSubFlag) {
			LineDetection(wall, 1.5, 2.0, -M_PI/6, M_PI/6, 0.2, 5.0, 30, true);
			if (wall.size() == 1) {
				break;
			}
		}
		ScanSubFlag = false;
		ros::spinOnce();
		looprate.sleep();
	}

	// confronting
	cout<<"[step2]  Confront."<<endl;
	Spur_set_angvel(0.4);
	Spur_spin_FS(wall[0].theta);
	sleep(6);

	// go forward
	while (ros::ok()) {
		if (ScanSubFlag) {
			if (0.1<front_dist()) {
				LineDetection(wall, 0.3, 2.0, -M_PI/12, M_PI/12, 0.2, 5.0, 30, true);
				if (wall.size() == 1) {
					Spur_line_FS(0.0, 0.0, wall[0].theta);
				}
			}
			else {
				Spur_stop();
				break;
			}
		}
		ScanSubFlag = false;
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
