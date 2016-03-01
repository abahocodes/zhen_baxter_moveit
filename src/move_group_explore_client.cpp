/**
 * \file move_group_explore_client.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief define explore poses on a grid base, and send grasp pose requests to move_group_explore_server
 */

#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cstdlib>

#include <move_to_target_pose.h>
#include <explore_reach.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_explore_client");
	
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<zhen_baxter_moveit::explore_reach>("explore_reach");
	ros::ServiceClient home_client = n.serviceClient<zhen_baxter_moveit::move_to_target_pose>("move_to_target_pose");
	zhen_baxter_moveit::explore_reach srv;
	zhen_baxter_moveit::move_to_target_pose home_srv;
	
	// GRASP BLOCK FROM ABOVE 
	// 	- Translation: [0.000, 0.060, 0.116]
	// 	- Rotation: in Quaternion [0.590, -0.399, 0.607, -0.354]
	// 	- Translation: [-0.004, 0.103, 0.104]
	// 	- Rotation: in Quaternion [0.667, -0.268, 0.632, -0.291]
	
	//explore behavior: explore within the vicinity of an object
	// 	double x_min = 0.0, x_max = 0.1, y_min = 0, y_max = 0.1, z_min = 0.10, z_max = 0.15;
	// 	double theta_x_min = 0, theta_x_max = 0, theta_y_min = M_PI/2.0, theta_y_max = M_PI/2.0, theta_z_min = -M_PI*3.0/4.0, theta_z_max = -M_PI/2.0;
	
	double x_min = 0.0, x_max = 0.0, y_min = 0.0, y_max = 0.0, z_min = 0.0, z_max = 0.0;
	double theta_x_min = 0.0, theta_x_max = 0.0, theta_y_min = M_PI*3.0/4.0, theta_y_max = M_PI*3.0/4.0, theta_z_min = -M_PI/2.0, theta_z_max = -M_PI/2.0;	
	
	// 	- Translation: [0.676, 0.535, 0.311]
	// 	- Rotation: in Quaternion [0.359, 0.852, -0.157, 0.347]
	// 	in RPY (radian) [-3.114, 0.781, 2.355]
	
	// 	double x_min = 0.676, x_max = 0.676, y_min = 0.535, y_max = 0.535, z_min = 0.311, z_max = 0.311;
	// 	double theta_x_min = -3.114, theta_x_max = -3.114, theta_y_min = 0.781, theta_y_max = 0.781, theta_z_min = 2.355, theta_z_max = 2.355;
	
	double x_res = 0.1, y_res = 0.1, z_res = 0.05;
	double theta_x_res = M_PI/4.0, theta_y_res = 10.0/180.0*M_PI, theta_z_res = M_PI/4.0;
	
	for(int x_sample=0; x_sample<=round((x_max-x_min)/x_res); x_sample++)
	{
		for(int y_sample=0; y_sample<=round((y_max-y_min)/y_res); y_sample++)
		{
			for(int z_sample=0; z_sample<=round((z_max-z_min)/z_res); z_sample++)
			{
				for(int theta_x_sample=0; theta_x_sample<=round((theta_x_max-theta_x_min)/theta_x_res); theta_x_sample++)
				{
					for(int theta_y_sample=0; theta_y_sample<=round((theta_y_max-theta_y_min)/theta_y_res); theta_y_sample++)
					{
						for(int theta_z_sample=0; theta_z_sample<=round((theta_z_max-theta_z_min)/theta_z_res); theta_z_sample++)
						{			
							printf("%d %d %d %d %d %d\n", x_sample, y_sample, z_sample, theta_x_sample, theta_y_sample, theta_z_sample);
							//Quaternion from euler angles: tf::Quaternion::setEuler(theta_y,theta_x,theta_z) this is correct, online doc is wrong
							tf::Vector3 euler_angle; 
							euler_angle.setX(theta_x_min+theta_x_sample*theta_x_res);  
							euler_angle.setY(theta_y_min+theta_y_sample*theta_y_res);
							euler_angle.setZ(theta_z_min+theta_z_sample*theta_z_res); 
							tf::Quaternion quat;
							// 							quat.setEuler(euler_angle.getY(), euler_angle.getX(), euler_angle.getZ());
							// 							quat.setEuler(euler_angle.getZ(), euler_angle.getY(), euler_angle.getX());	
							quat.setRPY(euler_angle.getX(), euler_angle.getY(), euler_angle.getZ());
							
							geometry_msgs::Quaternion odom_quat;
							tf::quaternionTFToMsg(quat, odom_quat);
							
							geometry_msgs::Pose explore_pose;
							explore_pose.orientation = odom_quat;
							explore_pose.position.x = x_min+x_sample*x_res; 
							explore_pose.position.y = y_min+y_sample*y_res;
							explore_pose.position.z = z_min+z_sample*z_res;
							printf("trans: %f %f %f\n", explore_pose.position.x, explore_pose.position.y, explore_pose.position.z);
							printf("euler: %f %f %f\n", euler_angle.getX(), euler_angle.getY(), euler_angle.getZ());
							printf("quat: %f %f %f %f\n", odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w);
							
							srv.request.target_pose=explore_pose; 
							srv.request.reference_frame="/block_link0";
							client.call(srv);
							ROS_INFO("explore pose succeed: %d", srv.response.succeed);
							
							// publish ROS tf static transform
							// 							tf::Transform transform;
							// 							transform.setRotation(tf::Quaternion(odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w));
							// 							transform.setOrigin(tf::Vector3(explore_pose.position.x, explore_pose.position.y, explore_pose.position.z));
							// 							static tf::TransformBroadcaster br;
							// 							
							// 							br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/block_link0", "/query_goal_pose"));
							// 							loop_rate.sleep(); //IMPORTANT!! otherwise somehow causes [Errno 110] Failed to get robot state on robot/state
							
							//homing...
							// 							std::cout << "homing...\n";
							// 							system("rosrun baxter_examples joint_position_file_playback_left_limb.py -f home_left_gripper.txt > temp.txt");
							// 							sleep(3.0);
							geometry_msgs::Quaternion home_quat;
							// 							tf::quaternionTFToMsg(tf::Quaternion(-0.698, -0.033, -0.373, 0.610), home_quat);
							tf::quaternionTFToMsg(tf::Quaternion(0.249, 0.860, -0.229, 0.382), home_quat);							
							geometry_msgs::Pose home_pose;
							home_pose.orientation = home_quat;
							// 							home_pose.position.x = 0.544;  
							// 							home_pose.position.y = 0.621;
							// 							home_pose.position.z = 0.342;
							home_pose.position.x = 0.621;  
							home_pose.position.y = 0.562;
							home_pose.position.z = 0.243;
							
							std::vector<geometry_msgs::Pose> home_poses;
							std::vector<std::string> reference_frames;
							
							home_poses.push_back(home_pose);
							reference_frames.push_back("/base");
							
							home_srv.request.target_poses=home_poses;
							home_srv.request.reference_frames=reference_frames;
							home_client.call(home_srv);
							ROS_INFO("homing succeed: %d", home_srv.response.succeed);
							
							// 							exit(1);
						}
					}
				}
			}
		}
	}
	
	// 	geometry_msgs::Quaternion odom_quat;
	// 	tf::quaternionTFToMsg(tf::Quaternion(0.667, -0.268, 0.632, -0.291), odom_quat);
	// 	
	// 	geometry_msgs::Pose explore_pose;
	// 	explore_pose.orientation = odom_quat;
	// 	explore_pose.position.x = -0.004;  
	// 	explore_pose.position.y = 0.103;
	// 	explore_pose.position.z = 0.104;
	
	// 	srv.request.target_pose=explore_pose; 
	// 	client.call(srv);
	// 	ROS_INFO("explore pose succeed: %d", srv.response.succeed);
	
	return 0;
}
