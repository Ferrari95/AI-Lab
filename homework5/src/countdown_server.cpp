#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <homework5/countdownAction.h>

class CountdownAction {

	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<homework5::countdownAction> as_;
		std::string action_name_;
		homework5::countdownFeedback feedback_;
		homework5::countdownResult result_;

	public:
		CountdownAction(std::string name) :
			as_(nh_, name, boost::bind(&CountdownAction::executeCB, this, _1), false),
			action_name_(name) {
			as_.start();
		}

		~CountdownAction(void) {
		}
		
		void executeCB(const homework5::countdownGoalConstPtr &goal) {
			bool success = true;
			
			ROS_INFO("%s: Executing, countdown of %i seconds", action_name_.c_str(), goal->n);
			
			ROS_INFO("Countdown: %d", goal->n);
			
			for(int i = goal->n - 1; i >= 0; i--) {
				if(as_.isPreemptRequested() || !ros::ok()) {
					ROS_INFO("%s: Preempted", action_name_.c_str());
					as_.setPreempted();
					success = false;
					break;
				}
				else {
					feedback_.currentCount = i;
					ros::Duration(1.0).sleep();
					ROS_INFO("Countdown: %d", i);
				}
			}

			if(success) {
				result_.finalCount = feedback_.currentCount;
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				as_.setSucceeded(result_);
			}
		}
};


int main(int argc, char** argv) {
	ros::init(argc, argv, "countdown_server");

	CountdownAction countdown("countdown_server");
	ros::spin();

	return 0;
}
