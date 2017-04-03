#include "ros/ros.h"
#include "stdlib.h"
#include "string.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

typedef const boost::function < void(const sensor_msgs::LaserScan::ConstPtr&) > callback;

class minimumClass {
	public:
		ros::Publisher pub;
		void minimum(const sensor_msgs::LaserScan::ConstPtr& msg);
};

void minimumClass::minimum(const sensor_msgs::LaserScan::ConstPtr& msg) {
	int size = msg->ranges.size();
	
	float min = msg->ranges[0];
	for(int i = 0; i < size; i++)
		if(msg->ranges[i] < min)
			min = msg->ranges[i];
	
	std_msgs::String toPublishMsg;
	toPublishMsg.data = std::to_string(min);
	this->pub.publish(toPublishMsg);
	
    ROS_INFO("Min: [%lf]", min);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "min_distance");
	ros::NodeHandle node;
	
	minimumClass mc;
	mc.pub = node.advertise<std_msgs::String>("min_distance_topic", 1000);
	callback boundedMinimum = boost::bind(&minimumClass::minimum, &mc, _1);
	
	ros::Subscriber sub = node.subscribe("base_scan", 1000, boundedMinimum);
	ros::spin();
	
	return 0;
}
