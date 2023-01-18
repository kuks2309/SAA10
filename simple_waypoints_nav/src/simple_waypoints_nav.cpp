#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/tf.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"

#define DEG2RAD(x) ((x)/180.*M_PI)
#define no_roi 8
/*
0.0 0.0
4.5 0.0
5.0 -1.0
5.0 -6.5
1.1 -7.2
*/
float   x[no_roi] = {0.5,  4.5,  5.0,   5.0,  1.1,  0.5, 1.68 , 3.0};
float   y[no_roi] = {0.1,  0.0, -1.0,  -6.4, -7.2, -5.5, -3.58 , -5.0};
float yaw[no_roi] = {0,      0, -90,   -90,   180,  90, 0 , 90};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Point3D
{
  float x;
  float y;
  float yaw;
};

int main(int argc, char** argv){
  int i;
  ros::init(argc, argv, "simple_navigation_nav_goals");
  ros::NodeHandle n;
  ros::Publisher marker_waypoint_pub = n.advertise<visualization_msgs::MarkerArray>("marker/waypoint", 1);
  Point3D p;
  std::vector<Point3D> vec_point;

  visualization_msgs::MarkerArray node_link_arr_waypoint;
  
  for(int i=0; i<no_roi; i++)
  {
    
    p.x = x[i];
    p.y = y[i];
    p.yaw = yaw[i];
    
    vec_point.push_back(p);   
  }
  
  for (size_t i = 0; i < no_roi; i++)
   { // 대충 포인트끼리 잇기 위한 for문
      
      visualization_msgs::Marker node_waypoint;
      node_waypoint.header.frame_id = "/map";
      node_waypoint.header.stamp = ros::Time::now();
      node_waypoint.id = (i+1)*100;
      node_waypoint.action = visualization_msgs::Marker::ADD;
      node_waypoint.pose.orientation.w = 1.0;
      // Points are green
      node_waypoint.color.g = 1.0f;
      node_waypoint.color.a = 1.0;
      node_waypoint.scale.x = 0.3;
      node_waypoint.scale.y = 0.3;
      node_waypoint.scale.z = 0.1;
      node_waypoint.type = visualization_msgs::Marker::SPHERE;
      node_waypoint.pose.position.x =  x[i]; //노드의 x 좌표
      node_waypoint.pose.position.y =  y[i]; //노드의 y 좌표 
      node_link_arr_waypoint.markers.push_back(node_waypoint);
   }
  
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
  
  for(i=0;i<8;i++)
  {
	  goal.target_pose.header.frame_id = "map";
	  goal.target_pose.header.stamp = ros::Time::now();
	  goal.target_pose.pose.position.x = x[i];
	  goal.target_pose.pose.position.y = y[i];
	  goal.target_pose.pose.position.z = 0.0;
	  
	  myQuaternion.setRPY(0,0,DEG2RAD(yaw[i]));
	  goal.target_pose.pose.orientation.x = myQuaternion.getX();
	  goal.target_pose.pose.orientation.y = myQuaternion.getY();
	  goal.target_pose.pose.orientation.z = myQuaternion.getZ();
	  goal.target_pose.pose.orientation.w = myQuaternion.getW();
	  
	  marker_waypoint_pub.publish(node_link_arr_waypoint);
	  ROS_INFO("Sending %d goal ",i);
	  
	  ac.sendGoal(goal);
	  ac.waitForResult();
	  
	  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  {
	     ROS_INFO("%3d Goal reached!",i);
	  }
      else
      {
		  ROS_INFO("The goal failed for some reason");
	  }
 }
  return 0;
}
