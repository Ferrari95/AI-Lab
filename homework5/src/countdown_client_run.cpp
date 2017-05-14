#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <homework5/countdownAction.h>

int main (int argc, char **argv) {
	ros::init(argc, argv, "countdown_client_run");
	
	actionlib::SimpleActionClient<homework5::countdownAction> ac("countdown_server", true);

	ROS_INFO("Waiting for action server to start.");
	
	ac.waitForServer();

	ROS_INFO("Action server started, sending goal.");
	
	homework5::countdownGoal goal;
	goal.n = 9;
	ac.sendGoal(goal);
	
	bool finished_before_timeout = ac.waitForResult(ros::Duration(double(goal.n) + 1.0));

	if(finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");
	
	return 0;
}
