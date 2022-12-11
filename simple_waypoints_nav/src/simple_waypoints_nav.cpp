#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>

#define DEG2RAD(x) ((x)/180.*M_PI)

/*
0.0 0.0
4.5 0.0
5.0 -1.0
5.0 -6.5
1.1 -7.2
*/
float   x[7] = {0.5,  4.5,  5.0,   5.0,  1.1,  0.5, 1.68};
float   y[7] = {0.1,  0.0, -1.0,  -6.5, -7.2, -5.5, -3.58};
float yaw[7] = {0,      0, -90,   -90,   180,  90, 0};
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  int i;
  ros::init(argc, argv, "simple_navigation_nav_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  ROS_INFO("Waiting for the move_base action server to connected");

  move_base_msgs::MoveBaseGoal goal;
  tf2::Quaternion myQuaternion;
  //we'll send a goal to the robot to move 1 meter forward
  
  for(i=0;i<7;i++)
  {
	  goal.target_pose.header.frame_id = "map";
	  goal.target_pose.header.stamp = ros::Time::now();
	  goal.target_pose.pose.position.x = x[i];
	  goal.target_pose.pose.position.y = y[i];
	  myQuaternion.setRPY(0,0,DEG2RAD(yaw[i]));
	  goal.target_pose.pose.orientation.x = myQuaternion.getX();
	  goal.target_pose.pose.orientation.y = myQuaternion.getY();
	  goal.target_pose.pose.orientation.z = myQuaternion.getZ();
	  goal.target_pose.pose.orientation.w = myQuaternion.getW();
	  ROS_INFO("Sending %d goal ",i);
	  ac.sendGoal(goal);
	  ac.waitForResult();
	  
	  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  {
	     ROS_INFO("Hooray, the base moved 1 meter forward");
	  }
      else
      {
		  ROS_INFO("The base failed to move forward 1 meter for some reason");
	  }
 }
  return 0;
}
