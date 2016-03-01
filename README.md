zhen_baxter_moveit
============
Get Baxter to pick & place in Gazebo, using Moveit!

This tutorial is under ROS indigo, Gazebo 2. All packages involved are going to be installed in ~/ros_gazebo_master.

1. Install baxter_simulator (refer to http://sdk.rethinkrobotics.com/wiki/Simulator_Installation) <br>
**NOTE**: if you have previously installed baxter SDK say in ~/ros_ws that works with the pysical robot, it's better to keep the installation of the baxter_simulator **separate** from the previous baxter SDK installation, e.g. install baxter_simulator in ~/ros_gazebo_master. 
  ```
  $ mkdir -p ~/ros_gazebo_master/src
  $ cd ~/ros_gazebo_master/src 
  $ wstool init . <br>
  $ wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/master/baxter_simulator.rosinstall
  $ wstool update
  $ source /opt/ros/indigo/setup.bash
  $ cd ~/ros_gazebo_master
  $ catkin_make
  $ catkin_make install
  $ cp src/baxter/baxter.sh .
  $ sudo apt-get install ros-indigo-xacro
  ```
  then edit the your_ip value and baxter_hostname in baxter.sh

2. Install Moveit! (refer to http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial) <br>
checkout Baxter MoveIt configuration package into ros_gazebo_master
  ```
  $ cd ~/ros_gazeob_master/src
  $ git clone https://github.com/ros-planning/moveit_robots.git
  $ sudo apt-get update
  $ sudo apt-get install ros-indigo-moveit-full
  $ cd ~/ros_gazebo_master
  $ catkin_make
  ```

3. Clone this repository (zhen_baxter_moveit) to ~/ros_gazebo_master/src, then
  ```
  $ cd ~/ros_gazebo_master/src/zhen_baxter_moveit
  $ mv baxter_table_single_block.world ../baxter_simulator/baxter_gazebo/worlds/ 
  $ mv home_left_gripper.txt ../../ 
  $ cd ~/ros_gazebo_master
  $ catkin_make
  ```
  in ros_gazebo_master/src/baxter_simulator/baxter_gazebo/baxter_world.launch, modifly line
  > \<arg name="world_name" value="$(find baxter_gazebo)/worlds/baxter.world"/\>
  
  into 
  
  > \<arg name="world_name" value="$(find baxter_gazebo)/worlds/baxter_table_single_block.world"/\>
  
4. Launch Gazebo with Baxter, Table, and block in the world, and move Baxter to home position 
  ```  
  $ roslaunch baxter_gazebo baxter_world.launch
  $ rosrun baxter_tools enable_robot.py -e
  $ rosrun baxter_examples joint_position_file_playback.py -f home_left_gripper.txt
  ```
  Launch move group for pick & place motion planning,
  ```
  $ roslaunch zhen_baxter_moveit move_group_server_simulator.launch
  ```
  Publish collision object to moveit planning scene based on gazebo scene
  ```
  $ rosrun zhen_baxter_moveit robot_block_simulator 0
  ```
  this node will publish the block and table as collision object in the moveit planning scene, and when the object is being
  grasped by the robot, this node will publish the block as an attached object to the robot, and if the object is released,
  this node will publish the block as a collision object again

5. (optional) Enable fake palmer-reflex to trigger gripper to close when at a grasping pose
  ```
  $ rosrun zhen_baxter_moveit fake_palmer_reflex
  ```

6. Run an example of pick up block at a pre-defined pose,
  ```
  $ rosrun zhen_baxter_moveit move_group_explore_client
  ```
  in either Gazebo or Moveit Rviz, you will see Baxter moves its left gripper to a grasp pose to the block, and retreive, and then move back to home pose.

PS: some detailed discussion about important parameters in Gazebo: https://github.com/RethinkRobotics/baxter_simulator/pull/52
