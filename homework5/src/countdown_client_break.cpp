#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <homework5/countdownAction.h>

int main (int argc, char **argv) {
	ros::init(argc, argv, "countdown_client_break");
	
	actionlib::SimpleActionClient<homework5::countdownAction> ac("countdown_server", true);
	
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();
	
	ac.cancelAllGoals();
	ROS_INFO("Goal deleted.");
	
	return 0;
}
