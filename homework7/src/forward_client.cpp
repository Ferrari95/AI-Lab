#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <homework7/forwardAction.h>

int main (int argc, char **argv) {
	ros::init(argc, argv, "forward_client");
	
	actionlib::SimpleActionClient<homework7::forwardAction> ac("forward_server", true);

	ROS_INFO("Waiting for action server to start.");
	
	ac.waitForServer();

	ROS_INFO("Action server started, sending goal.");
	
	homework7::forwardGoal goal;
	std::cout << "Insert desired speed: "; std::cin >> goal.desired_speed;
	std::cout << "Insert distance: "; std::cin >> goal.distance;
	
	ac.sendGoal(goal);
	
	bool finished_before_timeout = ac.waitForResult(ros::Duration(goal.distance / goal.desired_speed + 5.0));

	if(finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");
	
	return 0;
}
