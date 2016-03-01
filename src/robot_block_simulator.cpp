/**
 * \file robot_block_simulator.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief get block poses from Gazebo and publish the block frame through ROS tf static transform
 * 		  publish block (and table) as collision object, and attach/detach block to/from robot depending on whether the block is being grasped
 */

#include <ros/ros.h>
#include <gazebo_msgs/GetLinkState.h>
#include <tf/transform_broadcaster.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/collision_detection/collision_matrix.h>

int main(int argc, char** argv)
{
	if(argc<2)
	{
		printf("[Usage]: robot_block_simulator block_id\n");
		printf("supposed to work with ros_gazebo_master in simulator(not on physical robot)\n");
		exit(0);
	}
	
	// get the block link to be published
	int block_id = atoi(argv[1]);
	
	/***************************************
	 *  ros node initialization
	 ***************************************/
	std::stringstream ros_node_name; 
	ros_node_name << "robot_block_simulator" << block_id;
	ros::init(argc, argv, ros_node_name.str());
	ros::NodeHandle node_handle;
	ros::Rate loop_rate(100); 
	ros::Publisher collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
	ros::Publisher attach_collision_object_publisher = node_handle.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
	
	
	printf("waiting for listeners on topic /collision_object\n");
	while(collision_object_publisher.getNumSubscribers() < 1 && attach_collision_object_publisher.getNumSubscribers() < 1)
	{
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();    
	}
	
	/***************************************
	 *  setup fixed table msg (keep consistent with table object defined in 
	 *  ros_gazebo_gripper/src/baxter_simulator/baxter_gazebo/worlds/baxter_table.world, baxter_table_single_block.world)
	 ***************************************/
	moveit_msgs::CollisionObject collision_table;
	collision_table.header.frame_id = "/base";
	collision_table.id = "table";
	
	/* Define a table to add to the world. */
	shape_msgs::SolidPrimitive table_primitive;
	table_primitive.type = table_primitive.BOX;
	table_primitive.dimensions.resize(3);
	table_primitive.dimensions[0] = 0.7; //fixed X
	table_primitive.dimensions[1] = 2.0; //fixed Y
	table_primitive.dimensions[2] = 0.790;
	
	/* A pose for the table (specified relative to frame_id) */
	geometry_msgs::Pose table_pose;
	table_pose.position.x = 1.1;
	table_pose.position.y = 0.0;
	table_pose.position.z = -0.5275;
	
	collision_table.primitives.push_back(table_primitive);
	collision_table.primitive_poses.push_back(table_pose); 
	collision_table.operation = collision_table.ADD;
	
	collision_object_publisher.publish(collision_table);
	
	/***************************************
	 *  -setup moving block msg (keep consistent with table object defined in 
	 *  ros_gazebo_gripper/src/baxter_simulator/baxter_gazebo/worlds/baxter_table.world, baxter_table_single_block.world)
	 *  -publish base to block transform
	 ***************************************/
	// get block states from Gazebo
	ros::ServiceClient gazebo_client;
	gazebo_msgs::GetLinkState gazebo_request;
	
	gazebo_msgs::LinkState link_state;
	geometry_msgs::Pose link_pose;
	geometry_msgs::Point link_position;
	geometry_msgs::Quaternion link_quaternion;
	
	std::stringstream block_link_name; 
	block_link_name << "unit_box_" << block_id << "::link";
	gazebo_request.request.link_name = block_link_name.str();
	gazebo_request.request.reference_frame = "baxter::base";
	gazebo_client = node_handle.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
	
	std::stringstream block_frame_name; 
	block_frame_name << "/block_link" << block_id;
	
	moveit_msgs::CollisionObject collision_block;
	collision_block.header.frame_id = "/base";
	collision_block.id = block_frame_name.str();
	collision_block.operation = collision_block.ADD;
	
	/* Define a block primitive to add to the world. */
	shape_msgs::SolidPrimitive block_primitive;
	block_primitive.type = block_primitive.BOX;
	block_primitive.dimensions.resize(3);
	block_primitive.dimensions[0] = 0.025;
	block_primitive.dimensions[1] = 0.08; 
	block_primitive.dimensions[2] = 0.08; 
	
	collision_block.primitives.push_back(block_primitive);
	
	moveit_msgs::AttachedCollisionObject attached_object;
	attached_object.link_name = "left_wrist";
	attached_object.object.header.frame_id = "/base";
	attached_object.object.id = "box";
	attached_object.object.primitives.push_back(block_primitive);
	
	/*******************************************************************
	 * get end-effector gripping state 
	 * 1. if gripping=0, attached=false: update collision object pose and tf tree
	 * 2. if gripping=0, attached=true:  detach object, attached->false, then 1
	 * 3. if gripping=1, attached=false: attach object, attached->true
	 * 4. if gripping=1, attached=true:  do nothing
	 *******************************************************************/
	int gripping_state;
	bool attached = false;
	ros::Publisher planning_scene_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene",1);
	ros::ServiceClient moveit_client;
	moveit_client = node_handle.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
	moveit_msgs::GetPlanningScene moveit_request;
	moveit_msgs::PlanningSceneComponents moveit_request_type;
	moveit_request_type.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
	moveit_request.request.components = moveit_request_type;
	
	while(planning_scene_publisher.getNumSubscribers() < 1)
	{
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
	}
	
	while(ros::ok())
	{
		if (ros::param::get("/gripping", gripping_state))
		{
			if(gripping_state==0)
			{
				moveit_msgs::PlanningScene updated_planning_scene;
				updated_planning_scene.is_diff = true;
				
				if(attached)
				{
					//detach object: first detach object from robot; then re-add object into env
					ROS_INFO("detaching object");
					attached = false;
					attached_object.object.operation = attached_object.object.REMOVE;
// 					attach_collision_object_publisher.publish(attached_object);
					
					//add back collision checking for box and left_gripper_l_finger_tip, left_gripper_r_finger_tip
					if (moveit_client.call(moveit_request))
					{					
						moveit_msgs::AllowedCollisionMatrix acm;
						moveit_msgs::AllowedCollisionMatrix acm_modified;
						acm = moveit_request.response.scene.allowed_collision_matrix;
						
						collision_detection::AllowedCollisionMatrix acm_modifier(acm);
						acm_modifier.removeEntry("box", "l_gripper_l_finger_tip");
						acm_modifier.removeEntry("box", "l_gripper_r_finger_tip");
						
						acm_modifier.getMessage(acm_modified);
						
						updated_planning_scene.allowed_collision_matrix = acm_modified;
						updated_planning_scene.robot_state.attached_collision_objects.clear();
						updated_planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
// 						planning_scene_publisher.publish(updated_planning_scene);
					}else{
						ROS_ERROR("[moveit_request] Failed to get requested info");
						return 1;
					}
				}
				
				// add/update collision object pose and tf tree
				if (gazebo_client.call(gazebo_request))
				{
					//gazebo_request.response
					if(gazebo_request.response.success)
					{
						link_state = gazebo_request.response.link_state;
						link_pose = link_state.pose;
						link_position = link_state.pose.position;
						link_quaternion = link_state.pose.orientation;
						
						// 				printf("position: %f %f %f\n", link_position.x, link_position.y, link_position.z);
						// 				printf("quaternion: %f %f %f %f\n", link_quaternion.x, link_quaternion.y, link_quaternion.z, link_quaternion.w);
						
					}else{
						ROS_ERROR("[gazebo_request] Failed to get requested info");
						return 1;
					}
					
				}
				else
				{
					ROS_ERROR("Failed to call service gazebo_msgs::GetLinkState");
					return 1;
				}
				
				collision_block.primitive_poses.clear();
				collision_block.primitive_poses.push_back(link_pose);
				
				collision_block.operation = collision_block.ADD;
// 				collision_object_publisher.publish(collision_block);
				
				updated_planning_scene.world.collision_objects.clear();
				updated_planning_scene.world.collision_objects.push_back(collision_block);
				planning_scene_publisher.publish(updated_planning_scene);
				
				// publish ROS tf static transform
				tf::Transform transform;
				static tf::TransformBroadcaster br;
				transform.setRotation(tf::Quaternion(link_quaternion.x, link_quaternion.y, link_quaternion.z, link_quaternion.w));
				transform.setOrigin(tf::Vector3(link_position.x, link_position.y, link_position.z));
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base", block_frame_name.str()));
			}else if(gripping_state==1 && !attached)
			{
				//attach object: first remove object from env; then attach object to robot
				ROS_INFO("attaching object");
				attached = true;
				collision_block.operation = collision_block.REMOVE;
				collision_object_publisher.publish(collision_block);
				
				if (gazebo_client.call(gazebo_request))
				{
					//gazebo_request.response
					if(gazebo_request.response.success)
					{
						link_state = gazebo_request.response.link_state;
						link_pose = link_state.pose;
						link_position = link_state.pose.position;
						link_quaternion = link_state.pose.orientation;
						
						// 				printf("position: %f %f %f\n", link_position.x, link_position.y, link_position.z);
						// 				printf("quaternion: %f %f %f %f\n", link_quaternion.x, link_quaternion.y, link_quaternion.z, link_quaternion.w);
						
					}else{
						ROS_ERROR("[gazebo_request] Failed to get requested info");
						return 1;
					}
					
				}
				else
				{
					ROS_ERROR("Failed to call service gazebo_msgs::GetLinkState");
					return 1;
				}
				attached_object.object.primitive_poses.clear();
				attached_object.object.primitive_poses.push_back(link_pose);
				
				attached_object.object.operation = attached_object.object.ADD;
				attach_collision_object_publisher.publish(attached_object);
				
				//disable collision checking for box and left_gripper_l_finger_tip, left_gripper_r_finger_tip
				if (moveit_client.call(moveit_request))
				{					
					moveit_msgs::AllowedCollisionMatrix acm;
					moveit_msgs::AllowedCollisionMatrix acm_modified;
					acm = moveit_request.response.scene.allowed_collision_matrix;
				
					collision_detection::AllowedCollisionMatrix acm_modifier(acm);
					acm_modifier.setEntry("box", "l_gripper_l_finger_tip", true);
					acm_modifier.setEntry("box", "l_gripper_r_finger_tip", true);
					acm_modifier.getMessage(acm_modified);
					
					moveit_msgs::PlanningScene updated_planning_scene;
					updated_planning_scene.is_diff = true;
					updated_planning_scene.allowed_collision_matrix = acm_modified;
					planning_scene_publisher.publish(updated_planning_scene);
				}else{
					ROS_ERROR("[moveit_request] Failed to get requested info");
					return 1;
				}
			}
			
		}else
			ROS_ERROR("cannot get rosparam /gripping");
			
		loop_rate.sleep(); //IMPORTANT!! otherwise somehow causes [Errno 110] Failed to get robot state on robot/state
	}
	
	return 0;
}
