#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"          // boolean data
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"      // ultrasonic sensor message
#include "geometry_msgs/Twist.h"    // cmd_vel
#include "geometry_msgs/Pose2D.h"    // Pose2D
#include "nav_msgs/Odometry.h"

#define frequency_odom_pub 50   // hz

std_msgs::Bool flag_AEB;
std_msgs::Float32 delta_range;
std_msgs::Float32 old_sonar_range;
geometry_msgs::Twist cmd_vel_msg; 

float x,y;                                 // vehicle position[m/s]
float delta_x, delta_y;
float vx,vy;                               // vehicle velocity[m/s]
float aeb_collision_distance = 200;          // aeb engagementdistance[m]

void odomCallback(const nav_msgs::Odometry& msg)
{
	float old_x, old_y;
	old_x = x;
	old_y = y;
	ROS_INFO("%.2lf %.2lf", msg.pose.pose.position.x, msg.pose.pose.position.y); 
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;
	delta_x = x - old_x;
	delta_y = y - old_y;
	vx = delta_x * frequency_odom_pub;
	vy = delta_y * frequency_odom_pub;
}


void UltraSonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  ROS_INFO("Sonar Seq: [%d]", msg->header.seq);
  ROS_INFO("Sonar Range: [%f]", msg->range);
  
  aeb_collision_distance = vx *(0.7 + 0.1) * 0.22778 * 3.5;  // 1m/sec = 0.22778 km/h
  if(msg->range <= aeb_collision_distance)
  {
	  ROS_INFO("AEB_Activation!!");		    
	  flag_AEB.data = true; 
  }
  else
  {
	  flag_AEB.data = false; 
  }
}

void CarControlCallback(const geometry_msgs::Twist& msg)
{
  ROS_INFO("Cmd_vel : linear_x [%f]", msg.linear.x);  
  
  cmd_vel_msg = msg; 
  
  //cmd_vel_msg.linear.x /=10; 
  //ROS_INFO("Cmd_vel : linear_x [%f]", cmd_vel_msg.linear.x);
}

void UltraSonarCallback2(const sensor_msgs::Range::ConstPtr& msg)
{
  ROS_INFO("Sonar2 Seq: [%d]", msg->header.seq);
  ROS_INFO("Sonar2 Range: [%f]", msg->range);
}

int main(int argc, char **argv)
{
  int count = 0;
  float collision_distance  = 0.0;
   
  ros::init(argc, argv, "aeb_controller");

  ros::NodeHandle n;   
  
  std::string odom_sub_topic = "/ackermann_steering_controller/odom";
    
  ros::Subscriber sub = n.subscribe("/range", 1000, UltraSonarCallback);
  ros::Subscriber sonar_sub = n.subscribe("/RangeSonar1", 1000, UltraSonarCallback2);
  ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 10, &CarControlCallback);
  ros::Subscriber sub_odom = n.subscribe(odom_sub_topic, 10, &odomCallback);

  ros::Publisher pub_aeb_activation_flag= n.advertise<std_msgs::Bool>("/aeb_activation_flag", 1);
  ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 10);
  ros::Publisher pub_pose2d  = n.advertise<geometry_msgs::Pose2D>("/pose2d",10);
       
  ros::Rate loop_rate(10);  // 10
  
  geometry_msgs::Pose2D pose2d_msg; 
  
  while (ros::ok())
  {	 
	if((count%10)== 0)
	{
	  pub_aeb_activation_flag.publish(flag_AEB);
	}
	
	
	if(flag_AEB.data == true)
	{
		if(cmd_vel_msg.linear.x>0) cmd_vel_msg.linear.x = 0;		
		pub_cmd_vel.publish(cmd_vel_msg);
	}
	
	else 
	{
		pub_cmd_vel.publish(cmd_vel_msg);
	}
	
	//collision_distance = vx * (1/10.);
	
    ROS_INFO("Odom : [%6.3f %6.3f] m | Veloctiy : [%6.3f %6.3f] m/s", x,y,vx,vy); 
    ROS_INFO("Collision Distance : %6.3f",aeb_collision_distance); 
    
    pose2d_msg.x = x;
    pose2d_msg.y = y;
    pose2d_msg.theta = 0.0;
    ROS_INFO("Pose2D : %6.3f %6.3f %6.3f",pose2d_msg.x, pose2d_msg.y, pose2d_msg.theta); 
    pub_pose2d.publish(pose2d_msg);	

	loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }
  return 0;
}

/*
 * 
 *ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 10);
  
 * 
	
	
	* */
