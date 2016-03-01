/**
 * \file move_group_server.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief receive end_effector target pose request, plan and execute motion to move Baxter to the target pose
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"

#include <moveit/robot_state/robot_state.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "move_to_target_pose.h"
#include "baxter_core_msgs/EndEffectorCommand.h"

#include <ctime>

// (Optional) Create a publisher for visualizing plans in Rviz.
ros::Publisher display_publisher;
moveit_msgs::DisplayTrajectory display_trajectory;

// moveit::planning_interface::MoveGroup group("left_arm");
moveit::planning_interface::MoveGroup* group;

moveit::planning_interface::MoveGroup::Plan first_plan;
moveit::planning_interface::MoveGroup::Plan second_plan;

bool move_to(zhen_baxter_moveit::move_to_target_pose::Request  &req,
			 zhen_baxter_moveit::move_to_target_pose::Response &res)
{
	clock_t begin = std::clock();
	
	std::vector<geometry_msgs::Pose> target_poses = req.target_poses;
	std::vector<std::string> reference_frames = req.reference_frames;
	moveit::planning_interface::MoveGroup::Plan plan;
	bool move_success = true;
	bool plan_success = true;
	
	res.succeed = true;	
	bool retreive = false;
	int retreive_count = 0;
	
	for(int i=0; i<(int)target_poses.size();i++)
	{
		if(retreive)
		{
			i=0;
			retreive = false;
		}
		/**************************************************
		 * publish query goal state
		 **************************************************/
		// 		static tf::TransformBroadcaster br;
		// 		tf::Transform query_goal_pose;
		// 		query_goal_pose.setOrigin(tf::Vector3(target_poses[i].position.x, target_poses[i].position.y, target_poses[i].position.z));
		// 		query_goal_pose.setRotation(tf::Quaternion(target_poses[i].orientation.x,target_poses[i].orientation.y,target_poses[i].orientation.z,target_poses[i].orientation.w));
		// 		br.sendTransform(tf::StampedTransform(query_goal_pose, ros::Time::now(), req.reference_frames[i], "/query_goal_pose"));
		
		/**************************************************
		 * plan for each target pose
		 **************************************************/
		ROS_INFO("[move group server] target pose[%d]: %f, %f, %f, %f, %f, %f, %f", i, target_poses[i].position.x, target_poses[i].position.y, target_poses[i].position.z,
				 target_poses[i].orientation.x, target_poses[i].orientation.y, target_poses[i].orientation.z, target_poses[i].orientation.w);
		
		ROS_INFO("[move group server] current pose: %f, %f, %f, %f, %f, %f, %f", group->getCurrentPose().pose.position.x, group->getCurrentPose().pose.position.y,
				 group->getCurrentPose().pose.position.z, group->getCurrentPose().pose.orientation.x, group->getCurrentPose().pose.orientation.y,
				 group->getCurrentPose().pose.orientation.z, group->getCurrentPose().pose.orientation.w);
		
		moveit::planning_interface::MoveGroup::Plan plan;
		group->setPoseReferenceFrame(req.reference_frames[i]);
		group->setPoseTarget(target_poses[i]);
		group->setStartStateToCurrentState();
		
		ROS_INFO("[move group server] pose reference frame: %s", group->getPoseReferenceFrame().c_str());
		plan_success = group->plan(plan);
		
		// 		move_success = group->move();
		// 		move_success = group->asyncMove(); // do not use this, sequential motion planning won't be executed
		
		// if the plan for the first target_pose fails, try move to retreive pose first then replan
		if((!plan_success) & (i==0) & (retreive_count==0))
		{
			retreive_count++;
			
			tf::TransformListener listener;
			ros::Rate rate(10.0);
			bool receivedTransformation = false;
			tf::StampedTransform grasp_pose;
			while (!receivedTransformation)
			{
				receivedTransformation = true;
				try{
					listener.lookupTransform("/block_link0", "/left_gripper", ros::Time(0), grasp_pose);
				}
				catch (tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
					receivedTransformation = false;
				}
				rate.sleep();
			}
			
			tf::Transform offset;
			offset.setIdentity();
			offset.setOrigin(tf::Vector3(0,0,-0.1));
			
			tf::Transform retreive_pose;
			retreive_pose = grasp_pose*offset;
			
			geometry_msgs::Quaternion odom_quat;
			tf::quaternionTFToMsg(retreive_pose.getRotation(), odom_quat);
			
			geometry_msgs::Pose target_retreive_pose;
			target_retreive_pose.orientation = odom_quat;
			target_retreive_pose.position.x = retreive_pose.getOrigin().x();  
			target_retreive_pose.position.y = retreive_pose.getOrigin().y();
			target_retreive_pose.position.z = retreive_pose.getOrigin().z();
			
			group->setPoseReferenceFrame(req.reference_frames[i]);
			group->setPoseTarget(target_retreive_pose);
			group->setStartStateToCurrentState();
			
			moveit::planning_interface::MoveGroup::Plan extra_plan;
			plan_success = group->plan(extra_plan);
			if(!plan_success)
			{
				res.succeed = false;
				break;
			}else{
				move_success = group->execute(extra_plan);
				retreive = true;
			}
		}
		
		if(!plan_success)
		{
			res.succeed = false;
			break;
		}else
		{
			move_success = group->execute(plan);
		}
	}
	
	clock_t end = std::clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	ROS_INFO("[move group server] move group motion planning + motion execution takes time: %f seconds", elapsed_secs);
	
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_server");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	// calibration left grippers
	ros::Publisher gripper_publiser;
	gripper_publiser = node_handle.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 1);
	baxter_core_msgs::EndEffectorCommand gripper_command;
	gripper_command.id = 65664;
	std::stringstream gripper_command_args;
	gripper_command_args << "{\"holding_force\": " << 80 << ", \"velocity\": 50.0, \"dead_zone\": 5.0, \"moving_force\": 20.0}";
	gripper_command.args = gripper_command_args.str().c_str();
	gripper_command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
	gripper_publiser.publish(gripper_command);
	
	/* This sleep is ONLY to allow Rviz to come up */
	group = new moveit::planning_interface::MoveGroup("left_arm");
	group->setPlanningTime(2.0);
	group->setWorkspace(-2, -2, -2, 2, 2, 2);
	group->allowReplanning(false);
	group->setPlannerId("RRTConnectkConfigDefault"); // LazyRRTkConfigDefault; RRTStarkConfigDefault; RRTConnectkConfigDefault
	
	// We will use the :planning_scene_interface:`PlanningSceneInterface`
	// class to deal directly with the world.
	//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 
	//     display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	
	// Getting Basic Information
	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame: %s", group->getPlanningFrame().c_str());
	// We can also print the name of the end-effector link for this group.
	ROS_INFO("End-Effector link: %s", group->getEndEffectorLink().c_str());
	// default tolerance on goal position and orientation
	// 	group->setGoalPositionTolerance(0.01);
	// 	group->setGoalOrientationTolerance(0.01);
	ROS_INFO("Goal position tolerance: %f", group->getGoalPositionTolerance());
	ROS_INFO("Goal orientation tolerance: %f", group->getGoalOrientationTolerance());
	group->setGoalJointTolerance(0.001);
	ROS_INFO("Goal joint tolerance: %f", group->getGoalJointTolerance());
	
	ros::ServiceServer service = node_handle.advertiseService("move_to_target_pose", move_to);
	ROS_INFO("move_group_server ready");
	
	ros::waitForShutdown();
	
	return 0;
}
