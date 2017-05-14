#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <homework7/forwardAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

class ForwardAction {

	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<homework7::forwardAction> as_;
		std::string action_name_;
		homework7::forwardResult result_;
		nav_msgs::Odometry initial_odom_;
		ros::Subscriber sub_;
		ros::Publisher pub_;
		bool is_odom_acquired_ = false;

	public:
		ForwardAction(std::string name) :
			as_(nh_, name, boost::bind(&ForwardAction::executeCB, this, _1), false),
			action_name_(name) {
			sub_ = nh_.subscribe("odom", 10, &ForwardAction::odomCB, this);
			pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
			as_.start();
		}

		~ForwardAction(void) {
		}
		
		static double eulerianDist(geometry_msgs::Point p1, geometry_msgs::Point p2) {
			return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
		}
		
		void odomCB(const nav_msgs::Odometry::ConstPtr& msg) {
			result_.odom_pose = *msg;
			is_odom_acquired_ = true;
		}
		
		void executeCB(const homework7::forwardGoalConstPtr& goal) {
			bool success = true;
			
			ROS_INFO("%s: distance %lf, speed %lf", action_name_.c_str(), goal->distance, goal->desired_speed);
			
			geometry_msgs::Twist motion_msg;
			motion_msg.linear.x = goal->desired_speed;
			
			while(!is_odom_acquired_)
				ros::spinOnce();
			initial_odom_ = result_.odom_pose;
			
			while(eulerianDist(initial_odom_.pose.pose.position, result_.odom_pose.pose.pose.position) < goal->distance) {
				pub_.publish(motion_msg);
				ros::spinOnce();
				
				if(as_.isPreemptRequested() || !ros::ok()) {
					ROS_INFO("%s: Preempted", action_name_.c_str());
					as_.setPreempted();
					success = false;
					break;
				}
			}

			if(success) {
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				as_.setSucceeded(result_);
			}
		}
};


int main(int argc, char** argv) {
	ros::init(argc, argv, "forward_server");
	
	ForwardAction countdown("forward_server");
	ros::spin();

	return 0;
}
