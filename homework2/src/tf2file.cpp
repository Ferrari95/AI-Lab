#include "ros/ros.h"
#include "stdlib.h"
#include "string.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "iostream"
#include "fstream"

using namespace std;

typedef const boost::function < void(const sensor_msgs::LaserScan::ConstPtr&) > callback;

class prClass {
	public:
		tf::TransformListener * listener;
		std::string path;
		void printRow(const sensor_msgs::LaserScan::ConstPtr& msg);
};

void prClass::printRow(const sensor_msgs::LaserScan::ConstPtr& msg) {
	tf::StampedTransform transform;
	
	try {
		if((*(this->listener)).canTransform("/odom", "/base_laser_link", ros::Time(0), NULL)) {
			(*(this->listener)).lookupTransform("/odom", "/base_laser_link", ros::Time(0), transform);
			ofstream myfile;
			myfile.open(this->path, ios::out | ios::app | ios::binary);
			myfile << "LASER " << transform.stamp_.toSec();
			myfile << " (" << transform.getOrigin().x();
			myfile << ", " << transform.getOrigin().y();
			myfile << ", " << getYaw(transform.getRotation()) << ")\n";
			myfile.close();
		}
	}
	catch(tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf2file");
	tf::TransformListener l;
	ros::NodeHandle node;
	
	std::string directory = argv[0];
    directory.erase(directory.find_last_of('/') + 1);
	std::string path = directory + "LaserLocation2D.txt";
	
	ofstream myfile;
	myfile.open(path, ios::out);
	myfile << "LASER LOCATION 2D\n";
	myfile.close();
	
	prClass pr;
	pr.listener = &l;
	pr.path = path;
	callback boundedPrintRow = boost::bind(&prClass::printRow, &pr, _1);
	
	ros::Subscriber sub = node.subscribe("base_scan", 1000, boundedPrintRow);	
	
	ros::spin();
	
	return 0;
}
