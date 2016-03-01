/**
 * \file robot_block_gripping.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief in simulator, topic /robot/end_effector/left_gripper/state/gripping is constantly 0, and it doesn't respond when gripping happens
 *        thus here we set a ros parameter gripping to 1 or 0 depending on whether the position of the left gripper
 */

#include <ros/ros.h>
#include <baxter_core_msgs/EndEffectorState.h>

bool gripping;

void gripper_position_callback(const baxter_core_msgs::EndEffectorState gripper_position)
{
// 	ROS_INFO("get gripper position: %f", gripper_position.position);
	
	if(gripper_position.position>80)
		gripping = false;
	else if(gripper_position.position>5)
		gripping = true;
	else 
		gripping = false;
}

int main(int argc, char** argv)
{
	/***************************************
	 *  ros node initialization
	 ***************************************/
	ros::init(argc, argv, "robot_block_gripping");
	ros::NodeHandle node_handle;
	ros::Rate loop_rate(100); 
	
	// default initial gripping value = 0
	gripping = false;
	node_handle.setParam("gripping", 0);
	
	// listen to current position of left gripper
	ros::Subscriber gripper_position_subscriber = node_handle.subscribe<baxter_core_msgs::EndEffectorState>("/robot/end_effector/left_gripper/state", 1, gripper_position_callback);
	
	while(node_handle.ok())
	{
		if(gripping)
			node_handle.setParam("gripping", 1);
		else
			node_handle.setParam("gripping", 0);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
