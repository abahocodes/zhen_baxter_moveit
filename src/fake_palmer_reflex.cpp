/**
 * \file fake_palmer_reflex.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief when gripper pose relative to block is close to a known grasp pose, command gripper to close
 */

#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cstdlib>
#include "baxter_core_msgs/EndEffectorCommand.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_palmer_reflex");
	ros::NodeHandle node_handle;
	ros::Rate rate(1.0);
	
	tf::Quaternion quat_grasp;
	quat_grasp.setRPY(0.0, M_PI*3.0/4.0, -M_PI/2.0);
	
	ros::Publisher gripper_publiser;
	gripper_publiser = node_handle.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 1);
	baxter_core_msgs::EndEffectorCommand gripper_command;
	gripper_command.id = 131073;
	
	// get current left_gripper pose in reference frame block_link0
	tf::TransformListener listener;
	tf::StampedTransform transform;
	int gripping;
	
	while(node_handle.ok())
	{
		if (node_handle.getParam("gripping",gripping))
		{
			if(gripping==0)
			{
				bool receivedTransformation = false;
				while (!receivedTransformation)
				{
					receivedTransformation = true;
					try{
						listener.lookupTransform("block_link0", "left_gripper", ros::Time(0), transform);
					}
					catch (tf::TransformException ex){
						ROS_ERROR("%s",ex.what());
						ros::Duration(1.0).sleep();
						receivedTransformation = false;
					}
					rate.sleep();
				}
				
				double distance = 0.0;
				tf::Quaternion quat = transform.getRotation();
				distance += pow(transform.getOrigin().getX(),2);
				distance += pow(transform.getOrigin().getY(),2);
				distance += pow(transform.getOrigin().getZ(),2);
				distance += pow(quat.getX() - quat_grasp.getX(),2);
				distance += pow(quat.getY() - quat_grasp.getY(),2);
				distance += pow(quat.getZ() - quat_grasp.getZ(),2);
				distance += pow(quat.getW() - quat_grasp.getW(),2);
				distance = sqrt(distance);
// 				
				// close left grippers if current gripper pose is close to a pre-defined grasp pose
				if(distance < 0.02)
				{
					ROS_INFO("palmer-reflex triggered");
					std::stringstream gripper_command_args;
					gripper_command_args << "{\"position\": 0.0}";
					gripper_command.args = gripper_command_args.str().c_str();
					gripper_command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
					gripper_publiser.publish(gripper_command);
				}
			}
		}else
			ROS_ERROR("failed to get ros parameter \"gripping\"");
		
		rate.sleep();
	}
	
	return 0;
}
