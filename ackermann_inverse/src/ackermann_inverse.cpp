#include "std_msgs/String.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "math.h"

#define DEG2RAD(x) (M_PI/180.0)*x
#define RAD2DEG(x) (180.0/M_PI)*x

#define MAX_L_STEER  3.0
#define MAX_R_STEER  -3.0

#define MAX_L_STEER_ANGLE  30
#define MAX_R_STEER_ANGLE -30


std::string cmd_vel_topic;
float wheel_base;

float rqt_robot_steering_cmd_angular = 0.0;
float rqt_robot_steering_cmd_angular_old = 0.0;

float rqt_robot_steering_cmd_linear = 0.0;
float rqt_robot_steering_cmd_linear_old = 0.0;

float steering_angle_d  = 0.0;
float cmd_vel_linear  = 0.0;
float cmd_vel_angular = 0.0;

void rqt_robot_cmd_velCallback(const geometry_msgs::Twist& msg)
{
  float radius;
  rqt_robot_steering_cmd_angular = msg.angular.z;
  rqt_robot_steering_cmd_linear  = msg.linear.x ;
  
  cmd_vel_linear  = msg.linear.x ;
  cmd_vel_angular = msg.angular.z;
 
  if( (fabs(cmd_vel_linear)<1.0e-7)  || (fabs(cmd_vel_angular)<1.0e-7)  ) 
  {
	  steering_angle_d  = 0.0;
  }
  else
  {
	  radius = cmd_vel_linear/cmd_vel_angular;
	  steering_angle_d =  RAD2DEG(atan(wheel_base/radius));
	  if( (steering_angle_d>=180) &&  (steering_angle_d<360) )
	  {
		  steering_angle_d = 360 -steering_angle_d;
	  }
	  if( (steering_angle_d>-360) &&  (steering_angle_d< -180) )
	  {
		  steering_angle_d = 360 + steering_angle_d;
	  }
  }
  
  steering_angle_d = (steering_angle_d >= MAX_L_STEER_ANGLE) ? MAX_L_STEER_ANGLE : steering_angle_d;
  steering_angle_d = (steering_angle_d <= MAX_R_STEER_ANGLE) ? MAX_R_STEER_ANGLE : steering_angle_d;
  
  ROS_INFO("%6.3lf %6.3lf radius : %6.3lf Steering Angle : %6.1lf",cmd_vel_linear,  cmd_vel_angular,radius, steering_angle_d);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ackermann_inverse");
    ros::NodeHandle n;
    int duration = 20;
    
    wheel_base = 0.450;
    n.getParam("cmd_vel", cmd_vel_topic);
    n.getParam("wheel_base", wheel_base);
    
    std_msgs::Float32 cmd_steering_msg;      
    cmd_steering_msg.data  = 0;
    
    ros::Subscriber sub_cmd_vel = n.subscribe("/ackermann_steering_controller/cmd_vel",100,&rqt_robot_cmd_velCallback);
    ros::Publisher car_steer_control_pub_cmd = n.advertise<std_msgs::Float32>("/Car_Control_cmd/Vison_SteerAngle_Float16", 10);
   
    ros::Rate rate(duration);

    while (ros::ok())
    {	
		cmd_steering_msg.data = steering_angle_d;
		car_steer_control_pub_cmd.publish(cmd_steering_msg);
		ros::spinOnce();
		rate.sleep();       
	}
}
